#include "main.h"

//GLOBALS
volatile uint8_t Steps2Acc= 50;
volatile uint8_t accSteps= 0;
volatile uint16_t CurAcc[50];
volatile uint8_t StepsDelta = 0;
volatile uint16_t CurDelay = 0;
volatile uint8_t countPause= 0;

volatile int8_t Dir = 1;
volatile int8_t NextDir = 1;
volatile uint8_t CurPosition = 50;
volatile int16_t CurError = 0;

//L1,L2		//L1,L4		//L4,L3		//L2,L3
volatile int8_t StepStates[] = {0b11011000,0b10111000,0b10110100, 0b11010100};
volatile int8_t CurState = 0;


//EXTERNAL GLOBAL
extern uint8_t Parts[PARTS_SIZE];
extern volatile uint8_t countSort;
extern volatile char HALLSENSOR;//Needs to be false
extern volatile char DECELFLAG;
extern volatile char EXFLAG;
extern volatile char PAUSEFLAG;
extern volatile char TARGETFLAG;
extern volatile char SLIPFLAG;
extern volatile uint8_t Steps2Exit;

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



uint8_t stepUpdateError(void)
{
	if(SLIPFLAG)
	{
		if(abs(CurError)<DROP_REGION)//We may need to check the time since slip to see if the part fell
		{//Maybe a reduced drop region and a delay to ensure piece hits
			SLIPFLAG = 0;
			CurError = Parts[countSort] - CurPosition;
		}else
		{
			CurError = Parts[countSort-1] - CurPosition;
			
		}
	}else
	{
		CurError = Parts[countSort] - CurPosition;
	}
	
	
	if(CurError>100)
	{
		CurError = CurError - 200;
	}else if(CurError<-100)
	{
		CurError = CurError + 200;
	}
	
	if(abs(CurError) < Steps2Acc)//change if slowing down to quickly at zone; may cause oscillation
	{
		TARGETFLAG = 1;
	}else
	{
		TARGETFLAG = 0;
	}
	return 1;
}










uint8_t stepUpdateDir(void){
	//if(!DECELFLAG){
		if(CurError == 0)
		{// if stepper is at target
			if(CurDelay > (MAXDELAY-MINDELAY))
			{// if stepper can stop
				Dir = 0; //stop stepping
				TARGETFLAG = 0; //clear target flag
				return 1;	
			}else
			{//Decelerate stepper
				DECELFLAG = 1;
				return 0;
			}
		}else if((abs(CurError)>SPIN_ROUND_LIMIT) && (CurDelay<MAXDELAY))
		{//Next target is close in same direction and you are at speed don't change
			DECELFLAG = 0;
			if(Dir != 0)
			{//Keep direction
				NextDir = Dir;
			}else
			{//edge case where Dir might be zero
				Dir = 1;
				return 1;
			}
		}else
		{//Calculate closest direction
			NextDir = (CurError>0) - (CurError<0);	
		}

		if(NextDir == Dir)
		{//next direction is the same
			Dir = NextDir;
			return 1;
		}else if(CurDelay >= MAXDELAY)
		{//stepper is can change direction
			Dir = NextDir;
			return 1;
		}else
		{//Decelerate stepper to switch directions
			DECELFLAG = 1;
			return 0;
		}
	return 1;
}



uint8_t stepUpdateDelay(void)
{
	
	if(Dir==0)
	{//if stepper is not stepping:
		stepRes();//reset stepper
	}else if(TARGETFLAG || DECELFLAG || PAUSEFLAG)
	{//Decelerate if prompted
		CurDelay = CurDelay + CurAcc[accSteps];
		if (CurDelay > MAXDELAY)
		{
			if(PAUSEFLAG && (Steps2Exit<3))
			{
				
			}else
			{
				CurDelay = MAXDELAY;
				accSteps = 0;
				DECELFLAG = 0;
			}		     
		}else if(accSteps>0){
			accSteps--;
		}
	
	}else if(CurDelay>MINDELAY)
	{//Accelerate if able
		CurDelay = CurDelay -  CurAcc[accSteps];
		if (CurDelay <= MINDELAY || CurDelay > MAXDELAY)
		{//overflow protection
			CurDelay = MINDELAY;
		}
		if(accSteps<Steps2Acc)
		{//acceleration increase
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



