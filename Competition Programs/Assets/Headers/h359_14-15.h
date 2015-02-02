#ifndef __359_14-15_H__
#define __359_14-15_H__

#pragma systemFile

#include "Xander's Drivers\hitechnic-sensormux.h"
#include "Xander's Drivers\hitechnic-irseeker-v2.h"
#include "Xander's Drivers\hitechnic-gyro.h"
#include "Xander's Drivers\lego-ultrasound.h"	fzq

tHTSMUX SMUX;
tHTIRS2 ir_front;
//#define _ir_front	msensor_S3_4
tHTIRS2 ir_back;
//#define _ir_back	msensor_S3_3
tHTGYRO gyro;
//#define US				msensor_S3_1

int ir_front_dir[3];
int ir_back_dir[3];
int nCount = 0;

void initializeRobot();
void readIR();

#define DRIVER				rightWheel
#define DRIVEL				leftWheel
#define THRESHOLD			10
#define LEFT_OFFSET 	-10
#define RIGHT_OFFSET 	5

const ubyte DRAGGER_UP = 180;
const ubyte DRAGGER_DOWN = 150;

const ubyte FRONT_DRAGGER_UP = 0;
const ubyte FRONT_DRAGGER_DOWN = 255;

const ubyte SCORER_OPEN_CG = 180;
const ubyte SCORER_OPEN_RG = 65;
const ubyte SCORER_OPEN_AUTO = 220;
const ubyte SCORER_CLOSE = 255;

void initializeRobot(){
	bFloatDuringInactiveMotorPWM = false;
	initSensor(&ir_front, _ir_front);
	initSensor(&ir_back, _ir_back);
	initSensor(&gyro, _gyro);
	servoChangeRate[dragger] = 0;
	servoChangeRate[front_dragger] = 0;
	servoChangeRate[scorer] = 12;
}

void readIR(){
	readSensor(&ir_front);
	ir_front_dir[nCount] = ir_front.acDirection;
	readSensor(&ir_back);
	ir_back_dir[nCount] = ir_back.acDirection;
	nCount++;
}

bool busy = false;
long target_power = 0;
int target_distance = 0;

#endif //__359_14-15_H__
