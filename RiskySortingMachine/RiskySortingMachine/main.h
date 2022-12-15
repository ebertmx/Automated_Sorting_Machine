#ifndef MAIN_H
#define MAIN_H
//////////////////////////////////////////////////////////////////////////
//INCLUDES
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>
#include "lcd.h"

//////////////////////////////////////////////////////////////////////////
//FUNCTIONS: Function descriptions with definitions

//stepper.c
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

//control.h
void Motor_init(void);
uint8_t startMotor();
uint8_t runMotor();
uint8_t brakeMotor();
uint8_t stopMotor();
void motorTimerStart(void);
void motorTimerStop(void);

void ADC_Init(void);
uint8_t classify(uint16_t reflectVal);
uint8_t updateCount(uint8_t pos);


uint8_t debounce(uint8_t pin, uint8_t level, uint16_t checkNum);
uint8_t debouncePINJ(uint8_t pin, uint8_t level, uint16_t checkNum);
void dispPause(void);

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

//////////////////////////////////////////////////////////////////////////
//DEFINITIONS


#define PARTS_SIZE 100 //size of Parts array

//Minimum reflection values for parts
#define HI_Reflect 1010 //highest accepted reflection ADC value
#define B_Reflect 960
#define W_Reflect  800
#define S_Reflect 300
#define A_Reflect  0

//step positions for different part types
#define B_ID  50
#define W_ID 150
#define A_ID  100
#define S_ID  200
//////////////////////////////////////////////////////////////////////////
//STEPPER
//Step limit at which continue to rotate at full speed instead of turning around
#define SPIN_ROUND_LIMIT 90

//NOTE timing units are arbitrary and calibrated from testing
#define MAXACC 0x00F0 //Maximum acceleration of stepper
#define MINDELAY 0x02F0 //minimum delay between steps
#define MAXDELAY 0x0A00//Maximum delay between steps

//number of steps over which to increase acceleration to MAXACC
#define JERKSTEPS 4
//////////////////////////////////////////////////////////////////////////
//Calibrated Timing
//NOTE timing units are arbitrary and calibrated from testing

//Time part takes to hit bucket after exiting EX
#define DROP_TIME 18000 

//Time it take for part to hit bucket from entering EX while belt is moving
#define ENTER_DROP_TIME 22500
//Time it take for part to hit bucket from wihtin EX while belt is stopped
#define BRAKE_DROP_TIME 23000

//number of steps away from target the part is allowed to fall
//ex: if target is 50 and DROP_REGION = 14, part can drop between 64 and 36
#define DROP_REGION 14

//////////////////////////////////////////////////////////////////////////
//MOTOR
#define MOTOR_START_SPEED 150 // speed motor runs after initialization
#define MOTOR_SPEED 150 // speed motor runs just after runMotor() is called
#define MOTOR_SLOW_SPEED 100 // steady state speed of motor

//control the deceleration of motor from MOTOR_SPEED to MOTOR_SLOW_SPEED
#define MOTOR_TIMER 0x00F0
#define MOTOR_DEC 2
#define MOTOR_DEC_RATE  0x0003
#define MOTOR_ACC_TIME 100
#define MOTOR_START_DELAY 0x2400

//////////////////////////////////////////////////////////////////////////
//FILTERING
#define BOUNCECHECK 500 //number of consecutive TRUE reads for button debounce
#define NOISECHECK 8 //number of consecutive TRUE read for sensor noise filter

#define PARTTIME 5 //minimum time before accepting an exit read from OR (ms)
#define SORTTIME 5//minimum time before accepting an exit read from EX (ms)

#define REFRESH_PERIOD 50 //refresh period (ms) of LCD
#define RAMPDOWN_DELAY 2000 //delay used to clear back part of belt when RAMPDOWN button pushed

#endif