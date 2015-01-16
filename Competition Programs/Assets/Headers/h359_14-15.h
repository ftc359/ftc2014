#ifndef __359_14-15_H__
#define __359_14-15_H__

#pragma systemFile

void initializeRobot();

#define DRIVER				rightWheel
#define DRIVEL				leftWheel
#define THRESHOLD			10

const ubyte DRAGGER_UP = 0;
const ubyte DRAGGER_DOWN = 100;

const ubyte SCORER_OPEN_CG = 180;
const ubyte SCORER_OPEN_RG = 65;
const ubyte SCORER_CLOSE = 240;

void initializeRobot(){
	servoChangeRate[dragger] = 0;
	servoChangeRate[scorer] = 12;
	servo[dragger] = DRAGGER_UP;
	servo[scorer] = SCORER_CLOSE;
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

task bounceLift(){
	while(true){
		motor[lift] = 50;
		wait1Msec(500);
		motor[lift] = -15;
		wait1Msec(500);
	}
}

#endif //__359_14-15_H__
