#include "main.h"


//GLOBALS
volatile uint8_t countB =0;
volatile uint8_t countW =0;
volatile uint8_t countS=0;
volatile uint8_t countA =0;
volatile char PULSEFLAG=0;
volatile uint8_t motorDecSpeed =MOTOR_SPEED;

volatile uint16_t exitTime =0;
volatile uint16_t enterTime =0;

volatile uint16_t dropTime = DROP_TIME;
volatile uint16_t enterdropTime = ENTER_DROP_TIME;

volatile uint16_t motorTime_d = 0;

//EXTERNALS
extern volatile uint16_t runTime_d;
extern volatile char MOTORFLAG; //needs to be false
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


volatile uint8_t Steps2Exit = 0;
volatile uint8_t Steps2MIN = 0;


uint8_t CalcExitTime(void)
{
	if(HOLDFLAG)
	{
		return 0;
	}
	
	Steps2Exit = DROP_REGION - abs(CurPosition - Parts[countSort-1]);
	Steps2MIN = Steps2Acc-accSteps;
	
	if(((CurError*Dir)>0) || (CurDelay>=MAXDELAY))
	{		
		if(Steps2Exit<Steps2Acc)
		{
			exitTime = (CurDelay - MINDELAY)/2 * Steps2Exit;
		}else
		{
			exitTime = (CurDelay - MINDELAY)/2 * Steps2MIN + (Steps2Exit - Steps2Acc)*MINDELAY;	
		} 
	}else
	{
			exitTime = (MAXDELAY - CurDelay)/2 * (Steps2Acc)
						+(MAXDELAY-MINDELAY)/2 * Steps2Acc
						+ (Steps2Exit -(Steps2Acc-Steps2MIN))*MINDELAY;
		
	}

    if(exitTime<dropTime)
    {
	    return 1;
    }else
    {
	    return 0;
    }

}


volatile uint8_t Steps2Enter = 0;
volatile int16_t EXCurError=0;
volatile int16_t EnCurError=0;
uint8_t CalcEnterTime(void)
{        
	
	if(abs(CurError)<DROP_REGION)
	{
		return 0;
	}
	

	Steps2Enter = 	abs(CurError) - DROP_REGION;
	Steps2MIN = Steps2Acc-accSteps;
	
	if(Steps2Enter>40)
    {
            return 1;    
    }        
	
	if((CurError*Dir)>0 )
	{
		if(Steps2MIN> Steps2Enter)
		{
			
			enterTime = (CurDelay - MINDELAY)/2 * Steps2MIN;
                     	
		}else
		{
            enterTime =   (Steps2Enter- Steps2MIN);
            enterTime = enterTime*MINDELAY;               
			enterTime  += (CurDelay - MINDELAY)/2 * Steps2MIN;			
          
        }
	}else
	{
	
			enterTime = (Steps2Enter- Steps2MIN);
            enterTime = enterTime*MINDELAY;
			enterTime += (MAXDELAY - CurDelay)/2 * Steps2Acc;
            enterTime += (MAXDELAY-MINDELAY)/2 * Steps2Acc;  
	}
   
     
	 
    
	if(enterTime>enterdropTime)
	{
		return 1;
	}else
	{            
		return 0;
	}
}


uint8_t startMotor(){
	PORTB &= 0x80;
	PORTB |= 0b00001011;
	TCNT0 = 0;
	OCR0A = MOTOR_START_SPEED;
	//TCCR0B |= _BV(CS01);
	if(!MOTORFLAG)
	{
		MOTORFLAG = 1;
		motorTimerStart();
		OCR5A = 0x2400;
		motorTime_d = runTime_d;
	}
	TCNT5 = 0x0000;//restart max motor run time
	return MOTORFLAG;
}

uint8_t runMotor(){
	
	PORTB &= 0x80;
	PORTB |= 0b00001011;
	TCNT0 = 0;
	OCR0A = MOTOR_SPEED;
	
	if(!MOTORFLAG)
	{
		motorTimerStart();
		MOTORFLAG = 1;
		motorTime_d = runTime_d;
	}
		return MOTORFLAG;
	
}

uint8_t brakeMotor(){
	PORTB &= 0x80;
	PORTB |= 0b00001111;
	//TCCR0B &= ~_BV(CS01) & ~_BV(CS02)& ~_BV(CS00);
	MOTORFLAG = 0;
	return MOTORFLAG;
}

uint8_t stopMotor(){
	//TCCR0B &= ~_BV(CS01) &~_BV(CS02)&~_BV(CS00);
	PORTB = 0x00;
	MOTORFLAG = 0;
	return MOTORFLAG;
}


void motorTimerStart(void){
	TCCR5B |= _BV(WGM52); // Configure counter for CTC mode;
	OCR5A = MOTOR_TIMER; //1s timer
	TCNT5 = 0x0000; //Counter value register; Reset to 0
	TIMSK5 |= _BV(OCIE5A);  //Enable Interrupt
	TCCR5B |= _BV(CS52)| _BV(CS50);//Set prescaler to 1024
	TIFR5 |= _BV(OCF5A);//reset interrupt flag
	motorDecSpeed = MOTOR_SPEED;
}//mTimer_init


void motorTimerStop(void){
	TCCR5B &= ~_BV(CS52)& ~_BV(CS50);
}


ISR(TIMER5_COMPA_vect){
	//if motor needs to slow down
	motorDecSpeed -= MOTOR_DEC;
	OCR5A = MOTOR_DEC_RATE;
	if(motorDecSpeed < MOTOR_SLOW_SPEED){//if less than slowest motor speed
		motorDecSpeed = MOTOR_SLOW_SPEED; //set as lowest speed
		MOTORFLAG = 0;
		motorTimerStop();
	}
	TCNT0 = 0;
	OCR0A = motorDecSpeed;
}//ISR






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



uint8_t classify(uint16_t reflectVal){
	if(reflectVal >= B_Reflect){
		countB+=1;
		return B_ID;
	}
	else if((reflectVal >= W_Reflect)){
		countW+=1;
		return W_ID;
	}
	else if(reflectVal >= S_Reflect){
		countS+=1;
		return S_ID;
	}else
	{
		countA+=1;
		return A_ID;
	}
}





volatile uint16_t countCheck = 0;
volatile uint8_t mask = 0;

uint8_t debounce(uint8_t pin, uint8_t level, uint8_t checkNum){
	mask = (1<<pin); //create pin read mask
	level = (level<<pin);
	countCheck = 0;
	for(countCheck = 0; countCheck<checkNum; countCheck++)//read the pin a number of times
	{	
		if((PIND & mask)!=level)//if any of the reads are false
		{
			return 0;//return false	
		}
	}
		return 1;//return true	
}


uint8_t debouncePINJ(uint8_t pin, uint8_t level, uint8_t checkNum){
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





uint8_t getregion(uint8_t pos){
	
	if(pos>210){
		return 0x01;
	}else if(pos>160)
	{
		return 0x04;
	}else if(pos>110)
	{
		return 0x03;
	}else if(pos>60)
	{
		return 0x02;
	}else if(pos>10)
	{
		return 0x01;
	}else
	{
		return 0x04;
	}
}




void mTimer_init(){
	TCCR1B |= _BV(CS11);//Set prescaler to 8
	TCCR1B |= _BV(WGM12); // Configure counter for CTC mode;
	OCR1A = 0x03E8; //Set top value for Timer counter
}//mTimer_init



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



//Starts System Timer
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
void runTimerResume(void){
	TCCR4B |= _BV(CS42) | _BV(CS40);
}

//System Timer
ISR(TIMER4_COMPA_vect){

	runTime_d +=1;//add 1/1000 seconds to system time
	
}//ISR



//BAD ISR
ISR(BADISR_vect)
{
	PORTC = 0xFF;
	//mTimer(1000);
}//BADISR




//DISPLAY

void dispComplete (void)
{
	
		LCDClear();
		LCDWriteIntXY(0,0, countB, 2);
		LCDWriteString(",");
		LCDWriteInt( countA, 2);
		LCDWriteString(",");
		LCDWriteInt( countW, 2);
		LCDWriteString(",");
		LCDWriteInt(countS, 2);
		LCDWriteString("->");
		LCDWriteInt(countSort, 2);
		LCDWriteStringXY(0,1, "T=");
		LCDWriteInt(runTime_d/1000, 2);
		LCDWriteString( ".");
		LCDWriteInt(runTime_d%10 , 1);
		LCDWriteString("s Complete");
	
}

void dispStatus(void){
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


	//LCDWriteStringXY(0,1,"N=" );
	//LCDWriteInt(enterTime,8);
	//LCDWriteString(" X=");
	//LCDWriteInt(exitTime,6);	

LCDWriteIntXY(0, 1, CurPosition, 3);
LCDWriteStringXY(3,1, ">");
LCDWriteIntXY(4, 1, Parts[countSort], 3);
// 
// 	LCDWriteIntXY(8,1, PAUSEFLAG,1);
// 	LCDWriteInt(HOLDFLAG,1);
//  	LCDWriteInt(TARGETFLAG,1);
//  	LCDWriteInt(DECELFLAG,1);
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