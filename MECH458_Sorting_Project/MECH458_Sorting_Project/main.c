#include "main.h"
//FLAGS
volatile char HALLSENSOR=0;
volatile char ENABLE=1;
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
volatile uint8_t DropError = 30;
extern volatile int8_t NextDir;
volatile int8_t DoubleDir = 1;
volatile int16_t NextError = 0;
volatile uint8_t NextDropError = 0;

volatile uint8_t CurPosition = 50;
volatile int8_t Dir = 1;
volatile int16_t CurError = 0;

volatile uint16_t adcValue = 1023;
volatile uint16_t adcPart =1023;
volatile uint16_t adcTemp =1023;
volatile uint16_t adcDisp =1023;
volatile uint16_t countADC=0;

volatile uint16_t runTime_d =0;
volatile uint16_t startTime_d =0;
volatile uint16_t exitTime =0;
volatile uint16_t enterTime =0;

volatile uint16_t exitdropTime =DROP_TIME + 0x1000;
volatile uint16_t enterdropTime =DROP_TIME-0x1000;

volatile uint16_t ORTime_s=0;
volatile uint16_t EXTime_s=0;

//EXTERNALS
extern volatile uint8_t Steps2Acc;
extern volatile uint8_t countB;
extern volatile uint8_t countW;
extern volatile uint8_t countS;
extern volatile uint8_t countA;
extern volatile uint16_t CurDelay;
extern volatile uint8_t accSteps;

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
	LCDWriteIntXY(14,0, runTime_d/1000, 2);
	

	LCDWriteIntXY(0, 1, CurPosition, 3);
	LCDWriteStringXY(3,1, ">");
	LCDWriteIntXY(4, 1, Parts[countSort], 3);
	LCDWriteStringXY(8, 1,"D" );
	LCDWriteIntXY(9, 1, CurDelay/122, 2);//delay in ms
	LCDWriteIntXY(12, 1, adcDisp, 4);

}



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
	
	//EXT INTERRUPTS
	EICRA |= _BV(ISC01);//PAUSE
	EICRA |= _BV(ISC11) |_BV(ISC10);//OR
	EICRA |= _BV(ISC21);// | _BV(ISC20);//EX
	EICRA |= _BV(ISC31) | _BV(ISC30);//HE
	//EIMSK |= 0x0F; //Enable INT[0-3]


	ADC_Init();
	mTimer_init();
	stepTimer_init();

	
	//brakeMotor();
	InitLCD(LS_BLINK|LS_ULINE);
	LCDClear();
	
	EIMSK |= 0x08;
	sei();// Enable global interrupts

	//CALIBRATION
	
	stepCalibrate();
	mTimer(2000);
	
	//stepStart();//Activate the stepper motor
	//testStep();
	//while(1);
	cli();
	EIMSK |= 0x0F;
	EIMSK &= ~(0x08);
	Motor_init();
	sei();
	
	//MAIN OPERATION
	countPart=0;
	countSort = 0;

	//stepStart();//Activate the stepper motor
	startMotor();//Start Belt
	runTimerStart();//Start System Timer
	

	STANDBY:
	//Handle Specific Processes and Display Data

	
	
	while (ENABLE==1)
	{	
		dispStatus();
		mTimer(20);//Refresh Rate		
	}//while ENABLE
	
	
	
	//Pause
	if(ENABLE ==0){
		brakeMotor();
		runTimerStop();
		cli();
		while((PIND & 0x01) == 0x00);
		stepRes();

		while(!ENABLE){
			if(debounce(0,0,BOUNCECHECK)){

				while((PIND & 0x01) == 0x00);
				ENABLE = 1;
			}
		}//wait for ENABLE
		runTimerResume();
		sei();
		//EIMSK =intrSTATE;//set interrupt to saved state	
	}
	goto STANDBY;//goto standby state

	return(0);
}
//*************MAIN***************//



//*************ISR***************//

//ISR Stop Button
ISR(INT0_vect){
	if(debounce(0, 0, BOUNCECHECK)){
		if(ENABLE){
			ENABLE = 0;	
			brakeMotor();
		}
		else{
			runMotor();
			ENABLE = 1;
		}
	}
}//ISR Stop Button


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


//EX ISR //376 - 471 cycles
ISR(INT2_vect){
	
	
	if(!EXFLAG)
	{//Part is entering EX

			if(debounce(2, 0, NOISECHECK)){
		
				EXFLAG =1;//Part is at EX
				EIMSK &= ~_BV(INT2);
				EICRA |= _BV(ISC20);// Rising Edge
				EIMSK |= _BV(INT2); //Enable Interrupt
				
				if(abs(CurError)>DROP_REGION){
					brakeMotor();
				}
			//	enterTime  = (CurDelay - MINDELAY)/2 * (Steps2Acc - accSteps)
			//	+ (abs(CurError)- DROP_REGION - (Steps2Acc - accSteps))*MINDELAY;
				
// 				if(enterTime>enterdropTime){
// 						LCDWriteString("B");			
// 						brakeMotor();//Stop Belt
// 				}
				EXTime_s = runTime_d;
			}//LO
	}else
	{//Part is leaving EX
		if(debounce(2,1, NOISECHECK) && ((runTime_d - EXTime_s)>SORTTIME)	){

				EXFLAG = 0;
				EIMSK &= ~_BV(INT2);
				EICRA &= ~(_BV(ISC20));	//Turn on falling edge
				EIMSK |= _BV(INT2); //Enable Interrupt

				if(countSort<countPart)
				{
					countSort+=1;//go to next part immediately
				}
				
				exitTime  = (CurDelay - MINDELAY)/2 * (Steps2Acc - accSteps)
				 + ((DROP_REGION - abs(CurPosition - Parts[countSort-1])) - (Steps2Acc - accSteps))*MINDELAY;
				
					
				if(exitTime<exitdropTime)
				{
					//LCDWriteString("P");
					PAUSEFLAG = 1;
				}
				if(abs(CurError)>DROP_REGION){
					SLIPFLAG = 1;
				}	
		}//HI
	}
	EIFR |= _BV(INT2);
}//EX






//STEPPER ISR  //84 - 250 cycles
ISR(TIMER3_COMPA_vect){
//CONTROL STEPPER

	step();//step towards target
	
	if(!SLIPFLAG)
	{
		CurError = Parts[countSort] - CurPosition;
	
	}else
	{
		CurError = Parts[countSort-1] - CurPosition;
		if(abs(CurError)<DROP_REGION)
		{
			SLIPFLAG = 0;
		}
	}

	stepUpdateDir();
	stepUpdateDelay();
//CONTROL STEPPER


//CONTROL MOTOR
	if(!MOTORFLAG){//If motor is OFF
		
		if(abs(CurError)> DROP_REGION){
			enterTime  = (CurDelay - MINDELAY)/2 * (Steps2Acc - accSteps) + (abs(CurError)- DROP_REGION - (Steps2Acc - accSteps))*MINDELAY;
			if(enterTime<enterdropTime)
			{				
				runMotor();//Turn motor on
			}
			
		}else
		{
			runMotor();
		}
	}else 
	{
		if((abs(CurError)>DROP_REGION) && EXFLAG)
		{
			brakeMotor();
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


//********************FUNCTIONS*****************//











