#ifndef __359_14-15_H__
#define __359_14-15_H__

#pragma systemFile

void initializeRobot();

#define RIGHT_OFFSET	5
#define DRIVER				rightWheel
#define DRIVEL				leftWheel
#define THRESHOLD			10

const ubyte DRAGGER_UP = 100;
const ubyte DRAGGER_DOWN = 90;

const ubyte SCORER_OPEN = 155;
const ubyte SCORER_CLOSE = 245;

void initializeRobot(){
	servoChangeRate[dragger] = 0;
	servoChangeRate[scorer] = 12;
	servo[dragger] = DRAGGER_UP;
	servo[scorer] = SCORER_CLOSE;
}

long target_time = 0;
int target_power= 0;

enum SMstate{
	lift_stall = 0,
	lift_move = 1
};

SMstate SM_State = lift_stall;

void setSMstate(SMstate state, int power, long time){
	SM_State = state;
	target_power = power;
	target_time = time;
}

task state_machine(){
	long time_elapsed;
	while(true){
		switch(SM_State){
			case lift_stall:
				if(motor[lift] != 0)
					motor[lift] = 0;
				time_elapsed = 0;
				break;
			case lift_move:
				if(time_elapsed < target_time)
					motor[lift] = target_power;
				else{
					time_elapsed = 0;
					SM_State = lift_stall;
				}
				break;
			default:
				break;
		}
		wait1Msec(10);
		time_elapsed += 10;
	}
}

#endif //__359_14-15_H__
