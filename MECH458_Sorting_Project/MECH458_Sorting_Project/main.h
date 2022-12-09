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

uint8_t CalcExitTime(void);

uint8_t CalcEnterTime(void);


void dispStatus(void);
void dispComplete (void);
void dispFLAGS(void);

#define PARTS_SIZE 100


#define HI_Reflect 1010
#define B_Reflect 960
#define W_Reflect  800
#define S_Reflect 300
#define A_Reflect  0

#define B_ID  50
#define W_ID 150
#define A_ID  100
#define S_ID  200


#define SPIN_ROUND_LIMIT 90


//Test speed = 3510, one battery, fails with 2
#define MAXACC 0x00A0 //0x00FF
#define MINDELAY 0x0300//0x02A4
#define MAXDELAY 0x0A00//0x0900
#define JERKSTEPS 4//3

//EXIT times, Increase if you are dropping late
#define DROP_TIME 0x6000 

//ENTER times, Decrease if you are dropping early
#define ENTER_DROP_TIME 0x2000
#define BRAKE_DROP_TIME 0x3400
#define DROP_REGION 15


#define MOTOR_START_SPEED 220
#define MOTOR_SPEED 170
#define MOTOR_SLOW_SPEED 110

#define MOTOR_TIMER 0x00F0
#define MOTOR_DEC 2
#define MOTOR_DEC_RATE  0x0003
#define MOTOR_ACC_TIME 100

#define BOUNCECHECK 250
#define NOISECHECK 60



#define PARTTIME 30
#define SORTTIME 30

#define REFRESH_PERIOD 50
#define RAMPDOWN_DELAY 4000


#endif