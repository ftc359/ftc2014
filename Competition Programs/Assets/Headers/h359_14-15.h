#ifndef __359_14-15_H__
#define __359_14-15_H__

#pragma systemFile

#include "Xander's Drivers\hitechnic-sensormux.h"
#include "Xander's Drivers\hitechnic-irseeker-v2.h"
#include "Xander's Drivers\hitechnic-gyro.h"
#include "Xander's Drivers\lego-ultrasound.h"

tHTSMUX SMUX;
tHTIRS2 ir_front;
#define _ir_front	msensor_S2_1
tHTIRS2 ir_back;
#define _ir_back	msensor_S2_2
tHTGYRO gyro;
#define _gyro			msensor_S2_3
#define US				msensor_S2_4

int ir_front_dir[2];
long ir_front_str[2];
int ir_back_dir[2];
long ir_back_str[2];
int nCount = 0;

void initializeRobot();
void readIR();

#define DRIVER				rightWheel
#define DRIVEL				leftWheel
#define THRESHOLD			10

const ubyte DRAGGER_UP = 180;
const ubyte DRAGGER_DOWN = 150;

const ubyte SCORER_OPEN_CG = 180;
const ubyte SCORER_OPEN_RG = 65;
const ubyte SCORER_OPEN_AUTO = 200;
const ubyte SCORER_CLOSE = 240;

void initializeRobot(){
	bFloatDuringInactiveMotorPWM = false;
	initSensor(&ir_front, _ir_front);
	initSensor(&ir_back, _ir_back);
	initSensor(&gyro, _gyro);
	servoChangeRate[dragger] = 0;
	servoChangeRate[scorer] = 12;
}

void readIR(){
	int iTemp;
	readSensor(&ir_front);
	ir_front_dir[nCount] = ir_front.acDirection;
	for(int index = 1; index < 5; index++)
		iTemp = (ir_front.acValues[index-1] >= ir_front.acValues[index])?ir_front.acValues[index-1]:ir_front.acValues[index];
	ir_front_str[nCount] = iTemp;
	readSensor(&ir_back);
	ir_back_dir[nCount] = ir_back.acDirection;
	for(int index = 1; index < 5; index++)
		iTemp = (ir_back.acValues[index-1] >= ir_back.acValues[index])?ir_back.acValues[index-1]:ir_back.acValues[index];
	ir_back_str[nCount] = iTemp;
	nCount++;
}

bool busy = false;
long target_time = 0;
int target_power= 0;

task liftTask(){
	busy = true;
	motor[lift] = target_power;
	wait1Msec(target_time);
	motor[lift] = 0;
	busy = false;
}

void moveLift(int power, long time){
	target_power = power;
	target_time = time;
	startTask(liftTask);
}

#endif //__359_14-15_H__
