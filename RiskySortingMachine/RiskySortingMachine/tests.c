#include "main.h"
#include "lcd.h"


extern uint8_t Parts[PARTS_SIZE];
extern volatile uint8_t countSort;
extern volatile int16_t CurError;
extern volatile uint16_t runTime_d;
extern volatile char RAMPDOWN;
extern volatile char ENABLE;
extern volatile char DROPFLAG;
extern volatile char SORTFLAG;


void testStep(void){
	runTimerStart();//Start System Timer
	Parts[0] = 50;
	Parts[1] = 100;
	Parts[2] = 50;
	Parts[3] = 100;
	Parts[4] = 150;
	Parts[5] = 200;
	Parts[6] = 50;
	Parts[7] = 200;
	Parts[8] = 50;
	Parts[9] = 200;
	Parts[10] = 150;
	Parts[11] = 100;
	Parts[12] = 50;
	
	for(int i=0; i<12; i++){
		while(abs(CurError)!=0){
			dispStatus();
			mTimer(10);
		}
		countSort+=1;
		mTimer(100);
		//while(abs(CurError)=<10);
	}
	runTimerStop();
	LCDClear();
	LCDWriteInt(runTime_d, 4);
}



extern volatile uint8_t countPart;
extern volatile uint16_t adcDisp;
extern volatile uint16_t countADC;
extern volatile char ORFLAG;
extern volatile uint8_t motorDecSpeed;



volatile uint16_t testdropTime_d = 0;
void testDrop(void)
{

	while (!RAMPDOWN)
	{
		if(!ENABLE)
		{
			
			LCDClear();
			LCDWriteString("T = ");
			LCDWriteInt((runTime_d - testdropTime_d)>>4, 5);
			ENABLE = 1;
			
		}
		
		if(SORTFLAG)
		{
			testdropTime_d = runTime_d;
		}
		
	}//while ENABLE
	
	
}


void testBelt(void){
	
	while(countPart<12)
	{
		LCDClear();
		LCDWriteStringXY(0, 0, "ADC=");
		LCDWriteIntXY(3, 1, adcDisp, 4);
		LCDWriteStringXY(8, 0, "cnt=");
		LCDWriteIntXY(12, 1, countADC, 4);
		
		LCDWriteStringXY(0, 0, "P=");
		LCDWriteIntXY(2, 1, countPart, 2);
		LCDWriteStringXY(5, 0, "OR");
		LCDWriteIntXY(7, 1, ORFLAG, 1);
		
		LCDWriteStringXY(9, 0, "S=");
		float speed = 100/255;
		speed = speed*motorDecSpeed;
		LCDWriteIntXY(11, 1, speed, 3);
		
		
		
		mTimer(20);
	}
}


extern volatile char EXFLAG;
void testBrake(void){
	for(int i = 0; i<4; i++)
	{
		while(!EXFLAG){
			brakeMotor();
			mTimer(1000);
			runMotor();
			while(EXFLAG);
			brakeMotor();
			mTimer(1000);
			runMotor();
		}
	}
	
}


