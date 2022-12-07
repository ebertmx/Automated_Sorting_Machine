#ifndef MAIN_H
#define MAIN_H

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>
#include "lcd.h"

uint8_t step();
uint8_t stepUpdateError(void);
uint8_t stepUpdateDir(void);
uint8_t stepUpdateDelay(void);
uint8_t optimizeStepper();
void stepRes(void);
void stepTimer_init ();
void stepStart();
void stepStop();
int8_t stepCalibrate();
void stepCalcAcc();


void testStep();


void Motor_init(void);
uint8_t startMotor();
uint8_t runMotor();
uint8_t brakeMotor();
uint8_t stopMotor();
void motorTimerStart(void);
void motorTimerStop(void);

void ADC_Init(void);
uint8_t classify(uint16_t reflectVal);


uint8_t debounce(uint8_t pin, uint8_t level, uint8_t checkNum);
uint8_t debouncePINJ(uint8_t pin, uint8_t level, uint8_t checkNum);
uint8_t getregion(uint8_t pos);


void mTimer_init();
void mTimer(int count);

void runTimerStart(void);
void runTimerStop(void);
void runTimerResume(void);

uint16_t calcExitTime(void);
uint16_t calcEnterTime(void);


void dispStatus(void);
void dispComplete (void);

#define HI_Reflect 1010
#define B_Reflect 950
#define W_Reflect  800
#define S_Reflect 400
#define A_Reflect  0

#define B_ID  50
#define W_ID 150
#define A_ID  100
#define S_ID  200

#define PARTS_SIZE 100


#define MAXACC 0x00E0
#define MINDELAY 0x02A8
#define MAXDELAY 0x0A00
#define JERKSTEPS 4


#define PAUSEDURATION 2

#define EXIT_DROP_TIME 0x78A8 //0.2s
#define RUNNING_EXIT_DROP_TIME 0x5000
#define ENTER_DROP_TIME 0x8000
#define RUNNING_ENTER_DROP_TIME 0x6000
#define DROP_REGION 22
#define DROP_ERROVF 160

#define MOTOR_START_SPEED 230
#define MOTOR_SPEED 180
#define MOTOR_SLOW_SPEED 150
#define MOTOR_MAX_SPEED 250
#define MOTOR_MIN_SPEED 40


#define MOTOR_TIMER 0x0180
#define MOTOR_DEC 1
#define MOTOR_DEC_RATE  0x0001

#define BOUNCECHECK 250
#define NOISECHECK 60


#define PARTTIME 5
#define SORTTIME 5


#define REFRESH_PERIOD 100
#define RAMPDOWN_DELAY 4000


#endif