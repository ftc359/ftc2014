#ifndef __359_14-15_H__
#define __359_14-15_H__

void initializeRobot();

const ubyte DRAGGER_UP = 110;
const ubyte DRAGGER_DOWN = 10;

const ubyte SCORER_OPEN = 90;
const ubyte SCORER_CLOSE = 0;

void initializeRobot(){
	servo[dragger] = DRAGGER_UP;
	servo[scorer] = SCORER_CLOSE;
}

#endif //__359_14-15_H__
