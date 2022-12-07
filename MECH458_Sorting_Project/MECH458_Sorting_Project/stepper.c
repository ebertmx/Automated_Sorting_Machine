#include "main.h"

//CONSTANTS
// const uint16_t MAXACC = 0x0080;//0x00D0;
// const uint16_t MINDELAY	=0x0380;//0x02A4;
// const uint16_t MAXDELAY = 0x0A00;//0x0A00;
// const uint16_t JERKSTEPS = 4;

//GLOBALS
volatile uint8_t Steps2Acc= 50;
volatile uint8_t accSteps= 0;
volatile uint16_t CurAcc[50];
volatile uint8_t StepsDelta = 0;
volatile uint16_t CurDelay = 0;
volatile uint8_t countPause= 0;

//L1,L2		//L1,L4		//L4,L3		//L2,L3
volatile int8_t StepStates[] = {0b11011000,0b10111000,0b10110100, 0b11010100};
volatile int8_t CurState = 0;


//EXTERNALS
extern volatile uint8_t CurPosition;
extern volatile int16_t CurError;
extern volatile int8_t Dir;
volatile int8_t NextDir = 1;
extern volatile uint16_t exitTime;
extern volatile uint16_t exitdropTime;


//EXTERNAL GLOBAL
extern uint8_t Parts[PARTS_SIZE];
extern volatile uint8_t countSort;
extern volatile char HALLSENSOR;//Needs to be false
extern volatile char DECELFLAG;
extern volatile char EXFLAG;
extern volatile char PAUSEFLAG;

uint8_t step(void){
	CurState = CurState + Dir;//Update CurState based on Direction
	//stepper roll over
	if (4 <= CurState){CurState = 0;}
	else if (-1 >= CurState){CurState = 3;}
		
	PORTA = StepStates[CurState]; //Step
	CurPosition = CurPosition + Dir;//Update CurPosition
	//protect against roll over
	if(CurPosition > 200 && Dir==1){CurPosition -=  200;}
	else if(CurPosition < 50 && Dir==-1){CurPosition += 200;}
	
	TCNT3 = 0x0000;//Reset Counter
	
	return 1;	//return step;
}//step


uint8_t stepUpdateDir(void){
	if(!DECELFLAG){
		if(CurError == 0)
		{
			if(CurDelay!= MAXDELAY){
				DECELFLAG = 1;
				return 0;
			}else
			{
				Dir = 0;
				return 1;
			}
		}else if(CurError>100)
		{//target is more than 100 steps CW
			NextDir = -1;//turn CCW
		}else if(CurError<(-100))
		{//target is more than 100 steps CCW
			NextDir = +1;//turn CW
		}else if((abs(CurError)<118) && (abs(CurError)>82))
		{//Next target is exactly 100 steps away
			if(Dir != 0){
				NextDir = Dir;//Keep direction
			}else
			{
				Dir = 1;
				return 1;
			}
		}else
		{//Calculate closest direction
			NextDir = (CurError>0) - (CurError<0);
			
		}

		//Set Direction or Decelerate
		if(CurDelay >= MAXDELAY){
			Dir = NextDir;
			return 1;
		}
		
		
		
		if(NextDir == Dir)
		{//stepper is slow or next direction is the same
			Dir = NextDir;
			return 1;
		}else
		{
			DECELFLAG = 1;
			return 0;
		}
	}
	return 1;
}



uint8_t stepUpdateDelay(void)
{
	if(PAUSEFLAG)
	{	
		
		exitTime  = (CurDelay - MINDELAY)/2 * (Steps2Acc - accSteps)
		+ ((DROP_REGION - abs(CurPosition - Parts[countSort-1])) - (Steps2Acc - accSteps))*MINDELAY;
		
		exitdropTime -=CurDelay;
		
		if(exitTime<exitdropTime){
			DECELFLAG = 1;
		}else
		{
				
			//LCDWriteString("R");
			exitdropTime =DROP_TIME;
			DECELFLAG = 0;
			PAUSEFLAG = 0;	
		}
	}
	
	if(Steps2Acc >= abs(CurError) || DECELFLAG)
	{

		CurDelay = CurDelay + CurAcc[accSteps];
		
		if (CurDelay > MAXDELAY)
		{
			CurDelay = MAXDELAY;
			accSteps = 0;
			DECELFLAG = 0;
		}else if(accSteps>0){
			accSteps--;
		}
		
	}else if(CurDelay>MINDELAY)
	{
		//Accelerate
		CurDelay = CurDelay -  CurAcc[accSteps];
		
		if (CurDelay <= MINDELAY || CurDelay > MAXDELAY)//overflow protection
		{
			CurDelay = MINDELAY;
		}
		if(accSteps<Steps2Acc){
			accSteps++;
		}
	}else
	{
		return 0;
	}
	
	
	
	OCR3A = CurDelay;//set the new delay
	return 1;
}



void stepRes(void){
	accSteps = 0;
	StepsDelta = 0;
	CurDelay = MAXDELAY;
}



void stepTimer_init (void)
{
	TCCR3B |= _BV(WGM32);//Set CTC mode
	OCR3A = 0xFFFF; //Clear compare register A
	TCNT3 = 0x0000; //Clear count register
	TIMSK3 |= _BV(OCIE3A);  //Enable Interrupt
	return;
} //stepTimer_init


void stepStart(void){
	TCNT3 = 0x0000;//Reset counter

	OCR3A = MAXDELAY;//Set compare value
	
	TCCR3B |= _BV(CS31) | _BV(CS30);//Enable Stepper with prescaler
		
	TIFR3 |= _BV(OCF3A);//Reset interrupt flag
	//StepsDelta = 0;//reset acceleration step counter
	CurDelay = MAXDELAY;//Reset CurDelay
}//stepStart


void stepStop(void){
	TCCR3B &= ~_BV(CS31);//Disable timer
	TCCR3B &=~_BV(CS30);
}//stepStop


int8_t stepCalibrate(void){
	stepCalcAcc();
	
	CurDelay = MAXDELAY;
	HALLSENSOR = 0;//reset HALLSENSOR
	CurPosition = 0;//set CurPosition
	Parts[0] = 50;//Set motor to spin 360

	stepStart();//Start stepTimer
		
	while(!HALLSENSOR){
		if(abs(CurError)<20 && !HALLSENSOR){
			CurPosition = 0;
		}
	//dispStatus();
	//mTimer(10);	
	}//Wait for hall sensor to trigger

	//EIMSK &= ~(0x08); //Disable HALLSENSOR interrupt
	Parts[0] = B_ID;
	//CurPosition = B_ID;//Calibrate the stepper
	//accSteps = 0;
	//mTimer(1000);
	return 1;
}

void stepCalcAcc(void){

	uint16_t JERK = MAXACC/JERKSTEPS;
	uint16_t steps = 0;
	uint16_t delay = MAXDELAY;

	CurAcc[steps] = 0;
	for(steps = 1; steps<JERKSTEPS; steps++){
		delay -=CurAcc[steps-1];
		CurAcc[steps] = CurAcc[steps-1]+JERK;
		if(CurAcc[steps]>MAXACC){
			CurAcc[steps] = MAXACC;
		}

	}//Increase Acc
	
	CurAcc[steps] = MAXACC;
	while((delay -MAXACC -JERK*JERKSTEPS*JERKSTEPS/2)>MINDELAY){
		
		delay -=CurAcc[steps-1];
		if(delay<MINDELAY){
			delay = MINDELAY;
		}
		steps++;
		CurAcc[steps] = MAXACC;
		//mTimer(1000);
	}//Constant Acc
	
	while(delay >MINDELAY){
		steps++;
		
		delay -=CurAcc[steps-1];
		if(JERK> CurAcc[steps-1]){
			CurAcc[steps] = 0;
			break;
			}else{
			CurAcc[steps] = CurAcc[steps-1]-JERK;
			
		}

	}//Decrease Acc
	
	Steps2Acc = steps;	
}



