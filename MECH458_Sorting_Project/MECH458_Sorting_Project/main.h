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
uint8_t updateMotor(void);
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

#define PARTS_SIZE 100


#define HI_Reflect 1010
#define B_Reflect 954
#define W_Reflect  800
#define S_Reflect 400
#define A_Reflect  0

#define B_ID  50
#define W_ID 150
#define A_ID  100
#define S_ID  200


#define SPIN_ROUND_LIMIT 90


//Test speed = 3510, one battery, fails with 2
#define MAXACC 0x00FF
#define MINDELAY 0x02A4
#define MAXDELAY 0x0900
#define JERKSTEPS 3


#define EXIT_DROP_TIME 0x5000 //0.2s
#define RUNNING_EXIT_DROP_TIME 0x5000
#define ENTER_DROP_TIME 0x5000
#define RUNNING_ENTER_DROP_TIME 0x5000
#define DROP_REGION 20


#define MOTOR_START_SPEED 210
#define MOTOR_SPEED 160
#define MOTOR_SLOW_SPEED 150

#define MOTOR_TIMER 0x0180
#define MOTOR_DEC 1
#define MOTOR_DEC_RATE  0x0001
#define MOTOR_ACC_TIME 100

#define BOUNCECHECK 250
#define NOISECHECK 8



#define PARTTIME 1
#define SORTTIME 0

#define REFRESH_PERIOD 50
#define RAMPDOWN_DELAY 4000


#endif