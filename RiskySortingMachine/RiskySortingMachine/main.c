#include "main.h"
//FLAGS
volatile char HALLSENSOR=0; //Set when HE sensor is active
volatile char ENABLE=1; //reset/set with PAUSE button push
volatile char RAMPDOWN=0;// Set with RAMPDOWN button push
volatile char EXFLAG=0;//Set when part is within EX sensor
volatile char ORFLAG = 1;//Reset when part is within OR sensor
volatile char MOTORFLAG = 0;//Set when the motor is running
volatile char DECELFLAG = 0;//Set when stepper must decelerate

/*Set when a part leave EX before the 
stepper has reached the corresponding drop zone*/
volatile char HOLDFLAG = 0;

/*Set when stepper is within Steps2Acc of the target position
 and the part is NOT already dropping into the bin */
volatile char TARGETFLAG = 0;

/*Set when the stepper need to slow down to allow a part to fall into the bin */
volatile char PAUSEFLAG = 0;

//Set when the belt motor speed need to be updated
volatile char CALCFLAG = 0;

//Set when a piece is off the belt and falling towards the bin
volatile char DROPFLAG = 0;

//Set when a part enters EX and need to be sorted
volatile char SORTFLAG = 0;


//GLOBALS



/*
This array records all the parts detected by the system.
The parts are stored as a stepper position. The array is
parsed by 2 counters which indicate how many parts have been
detected and how many have been sorted
*/
uint8_t Parts[PARTS_SIZE];



/*The 2 counters below need to 'roll over' when they approach PARTS_SIZE
The size of the array limits how many parts are stored in the system at any time*/


/* 
This counter indicates the number of parts scanned and classified
by the system. It is incremented every time a new part enters OR
*/
volatile uint8_t countPart =0;
/*
This counter indicates which part is currently being sorted.
It is incremented every time a part leaves EX.
*/
volatile uint8_t countSort =0;



//ADC variables
volatile uint16_t adcValue = 1023;
volatile uint16_t adcPart =1023;
volatile uint16_t adcTemp =1023;
volatile uint16_t adcDisp =1023;
volatile uint16_t countADC=0;


/*
Variables used to record timing of events based on the
system timer which updates runtTime_d every 1ms.
*/
volatile uint16_t runTime_d =0;
volatile uint16_t refreshTime =0;
volatile uint16_t rampTime_d =0;
volatile uint16_t ORTime_s=0;
volatile uint16_t EXTime_s=0;

//EXTERNALS
extern volatile uint8_t CurPosition;
extern volatile uint16_t dropTime;
extern volatile uint16_t exitTime;
extern volatile int16_t CurError;
extern volatile uint16_t enterTime;
extern volatile uint16_t dropTime;
extern volatile uint16_t CurDelay;
extern volatile uint16_t enterdropTime;
extern volatile int8_t Dir;



int main(int argc, char *argv[]){
//INITIALIZATION	
	//Limit Clock to 8MHz
	CLKPR = 0x80;
	CLKPR = 0x01;

	
	cli();//Disable Interrupts
	
	//GPIO setup
	DDRA = 0xFF; //OUTPUT for stepper
	DDRB = 0xFF; //OUTPUT for motor
	DDRC = 0xFF; //OUTPUT for LCD
	DDRD = 0x00;//INPUT for EX, OR, HE, PAUSE
	DDRJ &= ~_BV(PINJ0); //INPUT
	
	//EXT INTERRUPTS
	EICRA |= _BV(ISC01);//PAUSE
	EICRA |= _BV(ISC11) |_BV(ISC10);//OR
	EICRA |= _BV(ISC21);//EX
	EICRA |= _BV(ISC31) | _BV(ISC30);//HE
	
	PCICR |= _BV(PCIE1);//Enable PCINT1
	PCMSK1 |= _BV(PCINT9);//RAMPDOWN
	
	PCICR |= _BV(PCIE0);//Enable PCINT 0
	PCMSK0 |= _BV(PCINT4);//Motor Controller


	ADC_Init();
	mTimer_init();
	stepTimer_init();
	InitLCD(LS_BLINK|LS_ULINE);
	LCDClear();
	EIMSK |= 0x08;//Enable HE
	sei();// Enable global interrupts
	// Calculate the stepper acceleration profile and calibrate position
	stepCalibrate();
	EIMSK |= 0x07;//Enable OR, EX, and PAUSE
	EIMSK &= ~(0x08);//Disable HE
	
	//Initialize the belt motor PWM and pins
	Motor_init();
	

	//reset counters
	countPart=0;
	countSort = 0;
	startMotor();//Start Belt motor
	runTimerStart();//Start System Timer
	
//MAIN OPERATION

/************************************************************************/
/* MAIN OPERATION: The following code contained in the labels STANDBY, DISABLE, and SHUTDOWN
	handle the user interface which includes the LCD screen and 2 buttons. 
	
					It DOES NOT perform sorting or control any other hardware.
					
The sorting algorithm is handle entirely by ISR's which can be found below main() in this file.                                                                 */
/************************************************************************/


STANDBY://Display the system status and handle button push events

	while (1)
	{				
		if(ENABLE)
		{//if the system is enabled
			if((runTime_d-refreshTime)>REFRESH_PERIOD)
			{//if LCD needs to be updated
				dispStatus();//display system information
				refreshTime = runTime_d;//control refresh rate of LCD
			}	
		}else
		{//else go to disabled state
			goto DISABLE;
		}


		if(RAMPDOWN)
		{//if RAMPDOWN button is pushed
			if(countSort != countPart)
			{//if there are parts which need sorting
				rampTime_d = runTime_d;//reset RAMPDOWN timer
				
			}else if((runTime_d-rampTime_d)>RAMPDOWN_DELAY)
			{//else if the belt has been cleared of parts				
				goto SHUTDOWN;//shutdown the system
			}
		}
				
	}//while ENABLE
	
	
	
DISABLE:// when the ENABLE is reset by ISR(INT0_vect)
	brakeMotor();//stop the belt
	while((PIND & 0x01) == 0x00);//wait for PAUSE button to be released
	stepStop();//stop the stepper motor
	runTimerStop();//stop the run timer
	uint8_t INTState = EIMSK; //save current interrupt state
	
	//Disable all interrupts except PAUSE button
	EIMSK = 0x01;
	PCMSK1 &= ~_BV(PCINT9);
	PCMSK0 &= ~_BV(PCINT4);
	
	brakeMotor();//insure motor is stopped (for edge case)
	stepRes();//Reset the stepper acceleration
	dispPause();//Display pause information
	
	//Wait for PAUSE button to be pushed again
	while(!ENABLE);
	while((PIND & 0x01) == 0x00);//wait for PAUSE button release
	
	//return interrupts to previous state
	EIMSK = INTState;
	PCMSK1 |= _BV(PCINT9);
	PCMSK0 |= _BV(PCINT4);
	runTimerResume();//start system timer
	stepStart();//start stepper
	runMotor();//start motor
	
goto STANDBY;//return to STANDBY mode
	
	
SHUTDOWN://When RAMPDOWN has been pushed and no part is on belt
	cli();//disable all interrupts
	PORTB = 0x00;//disable belt motor
	PORTA = 0x00;//disable stepper motor
	dispComplete();//display complete information
	while(1)
	{
		//wait until hardware reset
	}
	
	
	
	return(0);
}
//*************MAIN***************//



//*************ISR***************//

/************************************************************************/
/* DESCRIPTION: This ISR handles events with the OR sensor.
Initially, the interrupt is triggered on a rising edge which occurs when 
a part enters OR. It is then configured for a falling edge to detect when
the part leaves. This cycle repeats.

This ISR controls detecting and classifying parts. It also starts the ADC.

Filters have been implemented to prevent errors due to noise in the system. One part makes use
the function debounce() to implmenent a software lowpass filter. Another part compares the time
since the part has entered against a minimum expected time to block potential double reads cause
rocking or sliding of the parts.

RUNTIME ~500cc
                                                                  */
/************************************************************************/
ISR(INT1_vect)
{//OR has triggered falling or rising edge
	if(ORFLAG)
	{//if Part is entering OR
		
	
		if(debounce(1, 1, NOISECHECK))
		{//FILTER noise
			
			ORFLAG  = 0; //Part has entered OR
			
			//set to falling edge
			EIMSK &= ~_BV(INT1);
			EICRA &= ~_BV(ISC10);
			EIMSK |= _BV(INT1); 

			//reset adc variables
			countADC = 0;
			adcPart = 1023;
		
			ADCSRA |=_BV(ADSC);//start first ADC conversion
			
			ORTime_s = runTime_d;//record time part entered
			EIFR |= _BV(INT1);//reset interrupt flag (for edge case)
		}//HI
		
	}else//!ORFLAG
	{//if Part is leaving OR
		

		if(debounce(1, 0, NOISECHECK) && ((runTime_d - ORTime_s) > PARTTIME))
		{//FILTER noise and double edge detection
			ORFLAG  = 1;//Part has cleared OR	
			
			//Turn on rising edge
			EIMSK &= ~_BV(INT1); 
			EICRA |= _BV(ISC10);
			EIMSK |= _BV(INT1); 

			adcDisp = adcPart;//set display ADC variable
			
			//FILTER bad reads from ADC
			if((adcPart<HI_Reflect) && countADC>50)
			{//if a reflect value was recorded and the adc got more than minimum reads
				Parts[countPart] = classify(adcPart);//classify the part and add to array
				Parts[countPart+1] = Parts[countPart];//Initialize next array index
				countPart +=1;//increment part counter
			}
			EIFR |= _BV(INT1);//reset interrupt flag (for edge case) 
		}//LO	
	}//else
}//OR



/************************************************************************/
/* DESCRIPTION: This ISR handles events with the EX sensor.
Initially, the interrupt is triggered on a falling edge which occurs when 
a part enters EX. It is then configured for a rising edge to detect when
the part leaves. This cycle repeats.

This ISR set flags and variable to effectively sort parts as they
reach the end of the belt. It also starts and brakes the belt in 2 cases.

This ISR also increment the countSort variable every time a part leaves the sensor.

Filters have been implemented to prevent errors due to noise in the system. One part makes use 
the function debounce() to implmenent a software lowpass filter. Another part compares the time
since the part has entered against a minimum expected time to block potential double reads cause
rocking or sliding of the parts.

RUNTIME ~800cc
                                                                  */
/************************************************************************/
ISR(INT2_vect){
	
	if(!EXFLAG)
	{//Part is entering EX
		
		if(debounce(2, 0, NOISECHECK))
		{//FILTER noise
				EXFLAG =1;
				// Turn on rising edge
				EIMSK &= ~_BV(INT2);
				EICRA |= _BV(ISC20);
				EIMSK |= _BV(INT2); //Enable Interrupt
				EIFR |= _BV(INT2);
				
				SORTFLAG = 1;//Part need sorting
				if(HOLDFLAG)
				{//if the previous part has not finished sorting
					brakeMotor();//stop the belt
					enterdropTime = BRAKE_DROP_TIME;//set the drop time
				}else
				{//else keep the belt moving
					enterdropTime = ENTER_DROP_TIME;//set the drop time
				}
				EXTime_s = runTime_d;//record time part entered
		}//LO
	}else
	{//Part is leaving EX
		
		if(debounce(2,1, NOISECHECK) && ((runTime_d - EXTime_s)>SORTTIME))
		{//FILTER noise and double edge detection
				EXFLAG = 0;
				//Turn on falling edge
				EIMSK &= ~_BV(INT2);
				EICRA &= ~(_BV(ISC20));
				EIMSK |= _BV(INT2);
				EIFR |= _BV(INT2);
                
				updateCount(Parts[countSort]);//Update the sorted count for display
				
				if(countSort<countPart)
				{//if still parts to sort
					countSort+=1;//go to next part
					TARGETFLAG =0;//New target; reset flag
				}
				
				if(abs(CurError)>DROP_REGION)
				{//if stepper hasn't reached the drop zone for previous part
					HOLDFLAG = 1;//set hold flag to keep moving to previous target position
				}else
				{//else start the belt to drop the part
					runMotor();
				}
				//reset flag
                PAUSEFLAG=0;
				SORTFLAG = 0;
				
				DROPFLAG = 1;//part is now dropping into the bin
				
				//record time for part to hit bucket. 
				//Correct for next time ISR(TIMER3_COMPA_vect) runs
				dropTime = DROP_TIME - (OCR3A - TCNT3);   
				     
			EXTime_s = runTime_d;//record time part exited	
		}//HI
	}	
}//EX



/************************************************************************/
/* DESCRIPTION: This is the primary ISR of the sorting system. This ISR runs 
every time a step is required from the stepper motor. Consequently, the faster
the stepper is moving, the more this ISR runs.

The ISR controls writing to the stepper driver, calculating the variable CurError,
calculating the best direction to turn, and determining the next delay held in CurDelay.
CurDelay, controls the speed and acceleration of the stepper.

The ISR also triggers the ISR for motor control, ISR(PCINT0_vect).

RUNTIME ~400cc                                                          */
/************************************************************************/
ISR(TIMER3_COMPA_vect){
//CONTROL STEPPER
	step();//step stepper and update the position
	stepUpdateError(); //calculate the new stepper position error (CurError)
	stepUpdateDir(); //update the stepper direction
	stepUpdateDelay(); //update the stepper speed
//CONTROL STEPPER

	//trigger motor controller.
	CALCFLAG = 1;
	PORTB ^= _BV(PINB4);
}//stepTimer






	



//ADC ISR
ISR(ADC_vect){

	//if ADC is lower than value
	adcTemp = ADCL;
	adcTemp+= (ADCH<<8);
	countADC+=1;
	
	if(adcTemp<adcPart){
		adcPart = adcTemp;// set value to ADC
	}
	
	if(!ORFLAG){
		ADCSRA |=_BV( ADSC);
	}
}//ADC


//HE ISR
ISR(INT3_vect){
	if(debounce(3, 1, NOISECHECK)){
		//stepStop();
		CurPosition = B_ID;
		HALLSENSOR= 1;
	}
}//HE


//ISR Stop Button
ISR(INT0_vect){
	if(debounce(0, 0, BOUNCECHECK)){
		if(ENABLE)
		{
			ENABLE = 0;
		}else
		{
			ENABLE = 1;
		}
	}
}//ISR Pause Button

ISR(PCINT1_vect)
{
	if(debouncePINJ(0, 1, BOUNCECHECK)){
		RAMPDOWN = 1;
		rampTime_d = runTime_d;	
	}
}//ISR Ramp Button


ISR(PCINT0_vect)
{
	if(CALCFLAG)
	{
		
		if(SORTFLAG ^ HOLDFLAG)
		{
			if(CalcEnterTime())
			{
				brakeMotor();
				enterdropTime = BRAKE_DROP_TIME;
			}else
			{
				SORTFLAG = 0;
				runMotor();
			}
		}else if(SORTFLAG && HOLDFLAG)
		{
			brakeMotor();
		}
		
		
		
		if(DROPFLAG)
		{
			if(dropTime<CurDelay)
			{
				DROPFLAG = 0;
				PAUSEFLAG = 0;
			}else
			{
				dropTime -=CurDelay;	
				if(CalcExitTime())
				{
					PAUSEFLAG = 1;
				}else
				{
					PAUSEFLAG = 0;
				}
			}	
		}	
	}
	CALCFLAG = 0;
	
}










