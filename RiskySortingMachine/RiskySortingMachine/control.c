/*
 * control.c
 *
 * Created: 2022-12-09
 * Author: Matthew Ebert V00884117; Scott Griffioen V00884133
 * For: Sorting Machine, MECH 458, University of Victoria
 *
 * Dependencies: main.h
 * 
 * BOARD: ATMEGA 2560
 *
 * Description: This file contains functions used in main.c
 *
 */

#include "main.h"


//GLOBALS
volatile uint8_t countB =0;//counts number of black parts sorted
volatile uint8_t countW =0;//counts number of white parts sorted
volatile uint8_t countS=0;//counts number of steel parts sorted
volatile uint8_t countA =0;//counts number of aluminum parts sorted

volatile uint8_t motorDecSpeed =MOTOR_SPEED;//motor speed while decelerating
volatile uint16_t exitTime =0;//time for stepper to exit drop zone
volatile uint16_t enterTime =0;//time for stepper to enter drop zone
volatile uint16_t dropTime = DROP_TIME;//time for part to hit bucket from leaving EX
volatile uint16_t enterdropTime = ENTER_DROP_TIME;//Time for part to hit bucket from entering EX
volatile uint16_t motorTime_d = 0;//current running time of motor


volatile uint8_t Steps2Exit = 0;//number of steps until stepper exits drop zone
volatile uint8_t Steps2MIN = 0;//number of steps until stepper is at max speed
volatile uint8_t Steps2Enter = 0;//number of steps until stepper enters drop zone

//EXTERNALS
extern volatile uint16_t runTime_d;
extern volatile char MOTORFLAG;
extern volatile uint8_t countPart;
extern volatile uint8_t countSort;
extern uint8_t Parts[PARTS_SIZE];
extern volatile uint16_t adcDisp;
extern volatile char PAUSEFLAG;
extern volatile char TARGETFLAG;
extern volatile char DECELFLAG;
extern volatile char HOLDFLAG;
extern volatile uint8_t Steps2Acc;
extern volatile uint16_t CurDelay;
extern volatile uint8_t accSteps;
extern volatile uint8_t CurPosition;
extern volatile int16_t CurError;
extern volatile int8_t Dir;
extern volatile char EXFLAG;


/************************************************************************/
/* DESCRIPTION: Initializes the PWM and pins for belt motor
                                                                  */
/************************************************************************/
void Motor_init(void){
	//Set control register to fast PWM mode
	//Clear OC0A on compare match, set at BOTTOM
	TCCR0A |= _BV(COM0A1) | _BV(WGM01)| _BV(WGM00);
	//prescale the timer by 8 (3.9kHz)
	TCCR0B |= _BV(CS01);//|_BV(CS00) ;
	//reset the output compare register flag
	TIFR0 |= _BV(OCF0A);
	//set the output compare register value
	OCR0A = 0;
	//Stop
	stopMotor();
}


/************************************************************************/
/* DESCRIPTION: This function calculates the time until the stepper motor exits
the drop region defined by DROP_REGION. The function uses the current speed
and direction of the stepper to calculate the EXACT time it will take for
the stepper to reach an edge of the drop region.

The function returns a 1 if the stepper motor will exit the drop region before the
current part hits the bucket. It returns a 0 if the parts will hit before the
stepper exits.

The part drop time is estimated based on calibrated variables and updated every time
ISR(PCINT0_vect) runs if necessary. This time is stored in the variable
dropTime.
NOTE: dropTime is an approximation and of the actual part drop time which will
vary with every drop.We cannot know the exact time the part will take to drop
since we do not know the precise speed and position of the part past the EX sensor.
                                                                  */
/************************************************************************/
uint8_t CalcExitTime(void)
{
	if(HOLDFLAG)
	{//if part is past EX
		return 0;
	}
	
	//Calculate the number of steps to edge of drop zone 
	Steps2Exit = DROP_REGION - abs(CurPosition - Parts[countSort-1]);
	//Calculate number of steps until maximum acceleration
	Steps2MIN = Steps2Acc-accSteps;
	
	if(((CurError*Dir)>0) || (CurDelay>=MAXDELAY))
	{//If the stepper is turning in the right direction or able to switch directions
		
		if(Steps2Exit<Steps2Acc)
		{// if stepper will exit before reaching max speed
			exitTime = (CurDelay - MINDELAY)/2 * Steps2Exit;
		}else
		{//if stepper will reach max speed before exiting drop region
			exitTime = (CurDelay - MINDELAY)/2 * Steps2MIN + (Steps2Exit - Steps2Acc)*MINDELAY;
		}
		
	}else
	{//if stepper is turning in the wrong direction
		
		exitTime = (MAXDELAY - CurDelay)/2 * (Steps2Acc)//time to decelerate
		+(MAXDELAY-MINDELAY)/2 * Steps2Acc//time to accelerate
		+ (Steps2Exit -(Steps2Acc-Steps2MIN))*MINDELAY;//time to reach edge at max speed after acceleration
	}

	if(exitTime<dropTime)
	{//if stepper will exit region before current part drops
		return 1;
	}else
	{//else
		return 0;
	}

}



/************************************************************************/
/* DESCRIPTION: This function calculates the time until the stepper motor enters
the drop region defined by DROP_REGION. The function uses the current speed
and direction of the stepper to calculate the EXACT time it will take for
the stepper to reach an edge of the drop region.

The function returns a 1 if the stepper motor will enter the drop region after the
current part hits the bucket. It returns a 0 if the parts will hit after the
stepper enters.

The enterdropTime is an approximation of the time it will take the part to hit the bucket
when it enters or is in EX. This variable is set to 1 of two values which are calibrated
outside of regular run time. enterdropTime is set to ENTER_DROP_TIME if the belt is running
when its drop sequence begins or to BRAKE_DROP_TIME if the belt is stopped in the same situation.
                                                                  */
/************************************************************************/
uint8_t CalcEnterTime(void)
{
	
	if(abs(CurError)<DROP_REGION)
	{//if stepper is within drop region
		return 0;
	}
	
	//calculate steps to enter drop region
	Steps2Enter = 	abs(CurError) - DROP_REGION;
	//calculate steps to reach max speed
	Steps2MIN = Steps2Acc-accSteps;
	
	if(Steps2Enter>40)
	{//if stepper is far from the drop region
	//this is needed else enterTime may exceed range of 16 bit number
		return 1;
	}
	
	if(((CurError*Dir)>0) || (CurDelay>=MAXDELAY))
	{//if stepper is moving in the right direction or can change directions
		
		if(Steps2MIN > Steps2Enter)
		{//if stepper will enter drop region before reaching max speed	
			enterTime = (CurDelay - MINDELAY)/2 * Steps2MIN;	
		}else
		{//if stepper will reach max speed before entering region
			//Note: Calculation is broken up to prevent truncation and overflow
			enterTime =   (Steps2Enter- Steps2MIN); //time to reach edge after acceleration
			enterTime = enterTime*MINDELAY;
			enterTime  += (CurDelay - MINDELAY)/2 * Steps2MIN;//time to accelerate
			
		}
		
	}else
	{
		
		enterTime = (Steps2Enter- Steps2MIN);
		enterTime = enterTime*MINDELAY;
		enterTime += (MAXDELAY - CurDelay)/2 * Steps2Acc;
		enterTime += (MAXDELAY-MINDELAY)/2 * Steps2Acc;
		
	}
	
	
	if(enterTime>enterdropTime)
	{//if part will drop before stepper reaches region
		return 1;
	}else
	{//if stepper will reach region before part drops
		return 0;
	}
}

/************************************************************************/
/* DESCRIPTION: Starts the motor belt if required. Sets up a timer which will
decelerate the motor when it reaches the MOTOR_START_DELAY value.
Reset the motor timer and counter variables. This function should only be called
on startup or after an extended pause. Use runMotor() in other cases.                                                            */
/************************************************************************/
uint8_t startMotor(){
	PORTB &= 0x80;
	PORTB |= 0b00001011;
	TCNT0 = 0;
	OCR0A = MOTOR_START_SPEED;
	//TCCR0B |= _BV(CS01);
	if(!MOTORFLAG)
	{
		MOTORFLAG = 1;
		motorTimerStart();//start the motor timer to handle deceleration
		OCR5A =MOTOR_START_DELAY ;//Initial delay of the motor before deceleration
		motorTime_d = runTime_d;
	}
	TCNT5 = 0x0000;//restart max motor run time
	return MOTORFLAG;
}


/************************************************************************/
/* DESCRIPTION: Runs belt motor is required. Starts motor timer if motor is off.
Sets MOTORFLAG to indicate belt is running.                                                          */
/************************************************************************/
uint8_t runMotor(){
	//set to run forward, leave PWM signal as is.
	PORTB &= 0x80;
	PORTB |= 0b00001011;
	TCNT0 = 0;//reset PWM counter
	OCR0A = MOTOR_SPEED;//set initial motor speed
	
	if(!MOTORFLAG)
	{
		//start motor time which handles deceleration
		motorTimerStart();
		MOTORFLAG = 1;
		motorTime_d = runTime_d;//save motor run time
	}
	return MOTORFLAG;
	
}


/************************************************************************/
/* DESCRIPTION: brakes belt motor. Resets MOTORFLAG to indicate motor
stopped.                                                      */
/************************************************************************/
uint8_t brakeMotor(){
	PORTB &= 0x80;
	PORTB |= 0b00001111;
	MOTORFLAG = 0;
	return MOTORFLAG;
}


/************************************************************************/
/* DESCRIPTION: stops belt motor. Resets MOTORFLAG to indicate motor
stopped.                                                      */
/************************************************************************/
uint8_t stopMotor(){
	PORTB = 0x00;
	MOTORFLAG = 0;
	return MOTORFLAG;
}

/************************************************************************/
/* DESCRIPTION: set up the motor timer which handles motor deceleration.                                                   */
/************************************************************************/
void motorTimerStart(void){
	TCCR5B |= _BV(WGM52); // Configure counter for CTC mode;
	OCR5A = MOTOR_TIMER; //1s timer
	TCNT5 = 0x0000; //Counter value register; Reset to 0
	TIMSK5 |= _BV(OCIE5A);  //Enable Interrupt
	TCCR5B |= _BV(CS52)| _BV(CS50);//Set prescaler to 1024
	TIFR5 |= _BV(OCF5A);//reset interrupt flag
	motorDecSpeed = MOTOR_SPEED;
}//mTimer_init

/************************************************************************/
/* DESCRIPTION: Disables the motor timer                                                     */
/************************************************************************/
void motorTimerStop(void){
	TCCR5B &= ~_BV(CS52)& ~_BV(CS50);
}


/************************************************************************/
/* DESCRIPTION: Decelerates the motor based on the control parameters.
When the motor timer is active, the motor will run for a given period. This ISR
will then run and slow down the motor by an increment. This will repeat until the
motor reaches a certain speed. This gives a motor response with an initial fast pulse before
rapidly decelerating to a constant speed.                                                     */
/************************************************************************/
ISR(TIMER5_COMPA_vect){
	//if motor needs to slow down
	motorDecSpeed -= MOTOR_DEC;
	OCR5A = MOTOR_DEC_RATE;
	if(motorDecSpeed < MOTOR_SLOW_SPEED)
	{//if less than slowest motor speed
		motorDecSpeed = MOTOR_SLOW_SPEED; //set as lowest speed
		MOTORFLAG = 0;
		motorTimerStop();//disable timer
	}
	//Change motor speed by adjusting the PWM
	TCNT0 = 0;
	OCR0A = motorDecSpeed;
}//ISR

/************************************************************************/
/* DESCRIPTION: Initializes the ADC                                                    */
/************************************************************************/
void ADC_Init(void){
	// Set reference voltage to internal 5V (need capacitor)
	ADMUX |= _BV(REFS0);
	// Set Channel to ADCC1 (channel 1)
	ADMUX |= _BV( MUX0);
	// Enable ADC
	ADCSRA |= _BV(ADEN);
	// Enable Interrupt;
	ADCSRA |= _BV( ADIE);
	//ADCSRA |= _BV(ADLAR);
	// Set up prescaler to 128
	ADCSRA |= _BV( ADPS0) | _BV( ADPS1);// | _BV( ADPS2);
}


/************************************************************************/
/* DESCRIPTION: Returns a part classification based on ADC value                                                    */
/************************************************************************/
uint8_t classify(uint16_t reflectVal){
	if(reflectVal >= B_Reflect){
		//countB+=1;
		return B_ID;
	}
	else if((reflectVal >= W_Reflect)){
		//countW+=1;
		return W_ID;
	}
	else if(reflectVal >= S_Reflect){
		//countS+=1;
		return S_ID;
	}else
	{
		//countA+=1;
		return A_ID;
	}
}





volatile uint16_t countCheck = 0;
volatile uint8_t mask = 0;


/************************************************************************/
/* DESCRIPTION: This function is used as a debounce and filter. Unlike other
debounce functionality this function DOES NOT use time delays. Instead, 
consecutive reads of a pin on PORTD are taken. If the read is true, the program
continues until number of reads equals checkNum. If the read if false, the program
immediately returns false to the call function. 

The logic behind this method take advantage of the fact that for this application,
sensors or buttons change state relatively slowly compared to the processor speed.
This means that should a button bounce and give a false reading when pushed,
the program will revisit the calling function again (since all inputs are interrupt based),
until button has stopped bouncing. 

In the case of noise, this function acts like a low pass filter. Any frequency that
changes value at a lower period than the time it take to run this function
(controlled by checkNum) will be discarded. Any frequency or signal which has a
period greater than this function time will be accepted.

EX: An active high button is pushed by the user. The input signal begins bouncing. 
The MCU reads a rising edge, enters this functions, and reads several true readings.
Then a falling edge happens and a false reading is received. The program can now safely
exit the calling interrupt because, since the button is actually pushed, there will 
be another rising edge to trigger the interrupt and enter the debounce function again.
When bouncing has stopped, the last rising edge will trigger the interrupt and
this function will be called and return a true reading after reading the now stable
input checkNum times.                                                */
/************************************************************************/
uint8_t debounce(uint8_t pin, uint8_t level, uint16_t checkNum){
	mask = (1<<pin); //create pin read mask
	level = (level<<pin);
	countCheck = 0;
	for(countCheck = 0; countCheck<checkNum; countCheck++)
	{//read the pin a number of times
		if((PIND & mask)!=level)//if any of the reads are false
		{
			return 0;//return false
			//the next edge will trigger interrupt if actually true
			//and this function will be called again
		}
	}
	return 1;//return true
}

//same as above but on PORTJ
uint8_t debouncePINJ(uint8_t pin, uint8_t level, uint16_t checkNum){
	mask = (1<<pin); //create pin read mask
	level = (level<<pin);
	countCheck = 0;
	for(countCheck = 0; countCheck<checkNum; countCheck++)//read the pin a number of times
	{
		if((PINJ & mask)!=level)//if any of the reads are false
		{
			return 0;//return false
		}
	}
	return 1;//return true
}

/************************************************************************/
/* DESCRIPTION: Updates part count based on input position value.
Usually called as updateCount(Parts[countSort]) to update the counter of the
part being sorted.                                                */
/************************************************************************/
uint8_t updateCount(uint8_t pos){
	
	if(pos==200)
	{
		countS++;
	}else if(pos==150)
	{
		countW++;
	}else if(pos==100)
	{
		countA++;	
	}else
	{
		countB++;	
	}
	return 1;
}



/************************************************************************/
/* DESCRIPTION: Initialize mTimer. This function is only used for debugging.                                                   */
/************************************************************************/
void mTimer_init(){
	TCCR1B |= _BV(CS11);//Set prescaler to 8
	TCCR1B |= _BV(WGM12); // Configure counter for CTC mode;
	OCR1A = 0x03E8; //Set top value for Timer counter
}//mTimer_init



/************************************************************************/
/* DESCRIPTION: This is a millisecond timer. This function is only used for debugging.                                                   */
/************************************************************************/
void mTimer(int count){
	int i; //counter for ms
	i = 0;
	TCNT1 = 0x0000; //Counter value register; Reset to 0
	TIFR1 |= _BV(OCF1A); //Set the OC interrupt flag by writing 1
	while(i<count){
		if((TIFR1 & 0x02) == 0x02){
			TIFR1 |= _BV(OCF1A);//reset interrupt flag
			i++;	//increment counter to count milliseconds
		}
	}
	return;
}//mTimer




/************************************************************************/
/* DESCRIPTION: Initialize system timer which counts in ms                                                  */
/************************************************************************/
void runTimerStart(void){
	TCCR4B |= _BV(WGM42); // Configure counter for CTC mode;
	OCR4A = 0x0007; //0.01s timer
	TCNT4 = 0x0000; //Counter value register; Reset to 0
	TIMSK4 |= _BV(OCIE4A);  //Enable Interrupt
	TCCR4B |= _BV(CS42)| _BV(CS40);//Set prescaler to 1024
	TIFR4 |= _BV(OCF4A);//reset interrupt flag
}//mTimer_init


//Stops System Timer
void runTimerStop(void){
	TCCR4B &= ~_BV(CS42)& ~_BV(CS40);
}

//resumes system timer
void runTimerResume(void){
	TCCR4B |= _BV(CS42) | _BV(CS40);
}

//Updates System time
ISR(TIMER4_COMPA_vect){
	runTime_d +=1;//add 1/1000 seconds to system time	
}//ISR



//BAD ISR
ISR(BADISR_vect)
{
	PORTC = 0xFF;
}//BADISR


//////////////////////////////////////////////////////////////////////////
//DISPLAY FUNCTIONS

void dispComplete (void)
{
	LCDClear();
	LCDClear();
	LCDWriteString("B  A  W  S  C");
	LCDWriteIntXY(0,1, countB, 2);
	LCDWriteString(" ");
	LCDWriteIntXY(0,1, countA, 2);
	LCDWriteString(" ");
	LCDWriteIntXY(0,1, countW, 2);
	LCDWriteString(" ");
	LCDWriteIntXY(0,1, countS, 2);
	LCDWriteString(" ");
	LCDWriteIntXY(0,1,countSort, 2);	
}

void dispStatus(void)
{
	LCDClear();
	LCDWriteIntXY(0, 0, countSort, 2);
	LCDWriteStringXY(2,0,"/");
	LCDWriteIntXY(3,0, countPart, 2);
	LCDWriteStringXY(5,0, "(");
	LCDWriteIntXY(6,0, countB, 1);
	LCDWriteIntXY(7,0, countA, 1);
	LCDWriteIntXY(8,0, countW, 1);
	LCDWriteIntXY(9,0, countS, 1);
	LCDWriteStringXY(10,0, ")");
	LCDWriteStringXY(12,0, "T");
	LCDWriteIntXY(13,0, runTime_d/100, 3);
	LCDWriteIntXY(0, 1, CurPosition, 3);
	LCDWriteStringXY(3,1, ">");
	LCDWriteIntXY(4, 1, Parts[countSort], 3);
	LCDWriteIntXY(12, 1, adcDisp, 4);

}

void dispFLAGS(void){
	LCDClear();
	LCDWriteString("M");
	LCDWriteInt(MOTORFLAG,1);
	LCDWriteString(" P");
	LCDWriteInt(PAUSEFLAG,1);
	LCDWriteString(" T");
	LCDWriteInt(TARGETFLAG,1);
	LCDWriteStringXY(0,1," ");
	LCDWriteString("D");
	LCDWriteInt(DECELFLAG,1);
	LCDWriteString(" S");
	LCDWriteInt(HOLDFLAG,1);
}

extern volatile char ROLLFLAG;
void dispPause(void)
{
	LCDClear();
	LCDWriteString("B  A  W  S  O");
	LCDWriteIntXY(0,1, countB, 2);
	LCDWriteString(" ");
	LCDWriteIntXY(0,1, countA, 2);
	LCDWriteString(" ");
	LCDWriteIntXY(0,1, countW, 2);
	LCDWriteString(" ");
	LCDWriteIntXY(0,1, countS, 2);
	LCDWriteString(" ");
	if(ROLLFLAG)
	{
		LCDWriteIntXY(0,1, countPart + (PARTS_SIZE- countSort), 2);	
	}else
	{
		LCDWriteIntXY(0,1, countPart - countSort, 2);	
	}
	
}
