#include "main.h"
//FLAGS
volatile char HALLSENSOR=0;
volatile char ENABLE=1;
volatile char RAMPDOWN=0;
volatile char EXFLAG=0;
volatile char ORFLAG = 1;
volatile char MOTORFLAG = 0;
volatile char EMPTYFLAG = 0;
volatile char DECELFLAG = 0;
volatile char SLIPFLAG = 0;
volatile char TARGETFLAG = 0;
volatile char PAUSEFLAG = 0;

//GLOBALS
uint8_t Parts[PARTS_SIZE];
volatile uint8_t countPart =0;
volatile uint8_t countSort =0;


volatile uint16_t adcValue = 1023;
volatile uint16_t adcPart =1023;
volatile uint16_t adcTemp =1023;
volatile uint16_t adcDisp =1023;
volatile uint16_t countADC=0;

volatile uint16_t runTime_d =0;
volatile uint16_t refreshTime =0;
volatile uint16_t rampTime_d =0;
volatile uint16_t exitTime =0;
volatile uint16_t enterTime =0;
volatile uint16_t exitdropTime = EXIT_DROP_TIME;
volatile uint16_t enterdropTime = ENTER_DROP_TIME;
volatile uint16_t ORTime_s=0;
volatile uint16_t EXTime_s=0;

//EXTERNALS
extern volatile uint8_t Steps2Acc;
extern volatile uint16_t CurDelay;
extern volatile uint8_t accSteps;
extern volatile uint8_t CurPosition;
extern volatile int16_t CurError;


int main(int argc, char *argv[]){

	CLKPR = 0x80;
	CLKPR = 0x01;
	//INITIALIZATION
	
	cli();//Disable Interrupts
	
	//GPIO setup
	DDRA = 0xFF; //OUTPUT
	DDRB = 0xFF; //OUTPUT
	DDRC = 0xFF; //OUTPUT
	DDRD = 0x00;//INPUT
	DDRJ &= ~_BV(PINJ0);
	
	//EXT INTERRUPTS
	EICRA |= _BV(ISC01);//PAUSE
	EICRA |= _BV(ISC11) |_BV(ISC10);//OR
	EICRA |= _BV(ISC21);// | _BV(ISC20);//EX
	EICRA |= _BV(ISC31) | _BV(ISC30);//HE
	
	PCICR |= _BV(PCIE1);//RAMPDOWN
	PCMSK1 |= _BV(PCINT9);


	ADC_Init();
	mTimer_init();
	stepTimer_init();
	InitLCD(LS_BLINK|LS_ULINE);
	LCDClear();
	
	EIMSK |= 0x08;
	sei();// Enable global interrupts

	//CALIBRATION
	
	stepCalibrate();
	mTimer(2000);
	//testStep();
	//while(1);
	//cli();
	EIMSK |= 0x07;
	EIMSK &= ~(0x08);
	Motor_init();
	//sei();
	
	//MAIN OPERATION
	countPart=0;
	countSort = 0;

	startMotor();//Start Belt
	runTimerStart();//Start System Timer
	

STANDBY:
	//Handle Specific Processes and Display Data
	while (1)
	{	
				
		if(ENABLE)
		{
			if((runTime_d-refreshTime)>REFRESH_PERIOD)
			{
				dispStatus();
				refreshTime = runTime_d;	
			}
			
		}else
		{
			goto DISABLE;
		}


		if(RAMPDOWN)
		{
			if(countSort != countPart)
			{
				rampTime_d = runTime_d;	
				
			}else if((runTime_d-rampTime_d)>RAMPDOWN_DELAY)
			{
				goto SHUTDOWN;
			}
		}
				
	}//while ENABLE
	
	
	
DISABLE:
	brakeMotor();
	
	while((PIND & 0x01) == 0x00);
	stepStop();
	runTimerStop();
	uint8_t INTState = EIMSK;
	EIMSK = 0x01;
	PCMSK1 &= ~_BV(PCINT9);
	
	stopMotor();
	stepRes();
	dispStatus();
	while(!ENABLE)
	{
	}
	
	while((PIND & 0x01) == 0x00);
	EIMSK = INTState;
	PCMSK1 |= _BV(PCINT9);
	runTimerResume();
	stepStart();
	runTimerResume();
	
goto STANDBY;
	
	
	
	
	
SHUTDOWN:
	cli();
	PORTB = 0x00;
	PORTA = 0x00;
	
	 dispComplete();
	while(1)
	{
		
	}
	
	
	
	return(0);
}
//*************MAIN***************//



//*************ISR***************//



//OR ISR
ISR(INT1_vect){
	if(ORFLAG){
		
		if(debounce(1, 1, NOISECHECK)){
			ORFLAG  = 0; //Part has entered OR
			EIMSK &= ~_BV(INT1);
			EICRA &= ~_BV(ISC10); //Falling Edge
			EIMSK |= _BV(INT1); // Enable Interrupt


			countADC = 0;
			adcPart = 1023;
		
			ADCSRA |=_BV(ADSC);
		
			motorTimerStart();//slow down motor on approach
			ORTime_s = runTime_d;

		}//HI
		
	}else//!ORFLAG
	{
		
		if(debounce(1, 0, NOISECHECK) && ((runTime_d - ORTime_s) > PARTTIME)){
			ORFLAG  = 1;//Part has cleared OR	
			EIMSK &= ~_BV(INT1); // Disable Interrupt
			EICRA |= _BV(ISC10);//Turn on rising edge
			EIMSK |= _BV(INT1); // Enable Interrupt

			adcDisp = adcPart;
			if((adcPart<HI_Reflect) && countADC>50){
				Parts[countPart] = classify(adcPart);//classify the part and add to the step position
				Parts[countPart+1] = Parts[countPart];//Initialize next array index
				countPart +=1;//increment part counter
			}
		}//LO	
		
	}//else
	EIFR |= _BV(INT1); 
}//OR




uint16_t calcEnterTime(void)
{
	
	return 1;
}

uint16_t calcExitTime(void)
{
	
	return 1;
}



//EX ISR //376 - 471 cycles
ISR(INT2_vect){
	
	
	if(!EXFLAG)
	{//Part is entering EX

			if(debounce(2, 0, NOISECHECK))
			{
				EXFLAG =1;//Part is at EX
				EIMSK &= ~_BV(INT2);
				EICRA |= _BV(ISC20);// Rising Edge
				EIMSK |= _BV(INT2); //Enable Interrupt
				
				stepUpdateError();
				enterTime  = (CurDelay - MINDELAY)/2 * (Steps2Acc - accSteps) 
								+ (abs(CurError)- DROP_REGION - (Steps2Acc - accSteps))*MINDELAY;
				
				if(MOTORFLAG)
				{
					if(enterTime>ENTER_DROP_TIME)
					{
						brakeMotor();//brake motor
					}
				}else
				{
					if(enterTime>RUNNING_ENTER_DROP_TIME)
					{
						brakeMotor();//brake motor
					}	
				}
				EXTime_s = runTime_d;
			}//LO
	}else
	{//Part is leaving EX
		if(debounce(2,1, NOISECHECK) && ((runTime_d - EXTime_s)>SORTTIME)	)
		{
				EXFLAG = 0;
				EIMSK &= ~_BV(INT2);
				EICRA &= ~(_BV(ISC20));	//Turn on falling edge
				EIMSK |= _BV(INT2); //Enable Interrupt

				if(countSort<countPart)
				{
					countSort+=1;//go to next part immediately
					TARGETFLAG =0;
				}
				
				if(abs(CurError)>DROP_REGION){
					brakeMotor();
					SLIPFLAG = 1;
				}else
				{
						exitTime  = (CurDelay - MINDELAY)/2 * (Steps2Acc - accSteps) 
									+ (DROP_REGION - (CurPosition - Parts[countSort-1]) - (Steps2Acc - accSteps))*MINDELAY;				
						
						if(MOTORFLAG)
						{
							if(exitTime<RUNNING_EXIT_DROP_TIME)
							{
								PAUSEFLAG = 1;
								exitdropTime = RUNNING_EXIT_DROP_TIME;
							}
						}else
						{
							if(exitTime<EXIT_DROP_TIME)
							{
								PAUSEFLAG = 1;
								exitdropTime = EXIT_DROP_TIME;
							}
						}
					
				}
				
				
			
				
		}//HI
	}
	EIFR |= _BV(INT2);
}//EX




//STEPPER ISR  //84 - 250 cycles
ISR(TIMER3_COMPA_vect){
//CONTROL STEPPER

	step();//step towards target
	stepUpdateError(); //calculate the stepper position error
	
	if(PAUSEFLAG)
	{
		
		exitTime  = (CurDelay - MINDELAY)/2 * (Steps2Acc - accSteps)
					+ (DROP_REGION - (CurPosition - Parts[countSort-1]) - (Steps2Acc - accSteps))*MINDELAY;
		
		exitdropTime -=CurDelay;
		
		if(exitTime>exitdropTime)
		{
			exitdropTime =EXIT_DROP_TIME;
			PAUSEFLAG = 0;
		}
	}
	
	stepUpdateDir(); //update the stepper direction
	stepUpdateDelay(); //update the stepper speed
//CONTROL STEPPER

	


//CONTROL MOTOR
	if(!MOTORFLAG){//If motor is OFF
		
		if(abs(CurError)> DROP_REGION){
			enterTime  = (CurDelay - MINDELAY)/2 * (Steps2Acc - accSteps) 
							+ (abs(CurError)- DROP_REGION - (Steps2Acc - accSteps))*MINDELAY;
			if(SLIPFLAG)
			{
				if(enterTime<RUNNING_ENTER_DROP_TIME)
				{				
					runMotor();//Turn motor on
				}
			}else
			{
				if(enterTime<ENTER_DROP_TIME)
				{
					runMotor();//Turn motor on
				}
			}
			
		}else
		{
			runMotor();
		}
	}
//CONTROL MOTOR	

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













