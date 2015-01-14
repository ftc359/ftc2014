#ifndef __TELEOP_FUNCS_H__
#define __TELEOP_FUNCS_H__

#pragma systemFile

#ifndef getJoystickSettings //Check to see if "JoystickDriver.c" is included
#include "JoystickDriver.c"
#endif

#ifndef RIGHT_OFFSET
#define RIGHT_OFFSET	0
#endif

#ifndef LEFT_OFFSET
#define LEFT_OFFSET		0
#endif

#define J1X1        joystick.joy1_x1
#define J1Y1        joystick.joy1_y1
#define J1X2        joystick.joy1_x2
#define J1Y2        joystick.joy1_y2
#define J1B(BUT)    joy1Btn(BUT)
#define J1TH        joystick.joy1_TopHat

#define J2X1        joystick.joy2_x1
#define J2Y1        joystick.joy2_y1
#define J2X2        joystick.joy2_x2
#define J2Y2        joystick.joy2_y2
#define J2B(BUT)    joy2Btn(BUT)
#define J2TH        joystick.joy2_TopHat

//Function Prototypes
bool toggleJ1B(int button, bool onPress);
bool toggleJ2B(int button, bool onPress);
int joystickExponential(int deadzone, int maxPower, int joystickCurrent);
int joystickLinear(int deadzone, int maxPower, int joystickCurrent);
int accelerate(int startSpeed, int stopSpeed, long maxTime, long currentTime);

void driveTank(int maxPower, bool exponential);
void driveArcade(int maxPower, bool exponential, int joy);

static unsigned short nIgnoreJ1B, nIgnoreJ2B;  //2 bytes to use as booleans via bitmasking

bool toggleJ1B(int button, bool onPress){
    if(J1B(button) && !(nIgnoreJ1B & (1<<(button-1))))
        nIgnoreJ1B |= (1<<(button-1));    //Mask that bit to true
    else if(!J1B(button) && (nIgnoreJ1B & (1<<(button-1)))){
        nIgnoreJ1B ^= (1<<(button-1));    //Mask that bit to false
        return true;
    }
    if(onPress && (nIgnoreJ1B & (1<<(button-1))))
    	return true;
    return false;
}
bool toggleJ2B(int button, bool onPress){
    if(J2B(button) && !(nIgnoreJ2B & (1<<(button-1))))
        nIgnoreJ2B |= (1<<(button-1));    //Mask that bit to true
    else if(!J2B(button) && (nIgnoreJ2B & (1<<(button-1)))){
        nIgnoreJ2B ^= (1<<(button-1));    //Mask that bit to false
        return true;
    }
    if(onPress && (nIgnoreJ2B & (1<<(button-1))))
    	return true;
    return false;
}
int joystickExponential(int deadzone, int maxPower, int joystickCurrent){
	if(joystickCurrent == 0) return 0;
	int polarity = joystickCurrent/abs(joystickCurrent);
	long maxPower2 = maxPower*maxPower;
	return polarity*(maxPower-sqrt(((abs(joystickCurrent)-127)*maxPower2)/(deadzone-127)));
}
int joystickLinear(int deadzone, int maxPower, int joystickCurrent){
	if(joystickCurrent == 0) return 0;
	int polarity = joystickCurrent/abs(joystickCurrent);
	return polarity*(maxPower-((abs(joystickCurrent)-127)*maxPower)/(deadzone-127));
}

void driveTank(int maxPower, bool exponential){
#if defined(__HOLO_H__) && defined(DRIVEFR) && defined(DRIVEFL) && defined(DRIVEBR) && defined(DRIVEBL) && defined(THRESHOLD)
    if(exponential){
        motor[DRIVEFL] = (abs(J1Y1) > THRESHOLD)?joystickExponential(THRESHOLD,maxPower,J1Y1) - LEFT_OFFSET:0;
        motor[DRIVEFR] = (abs(J1Y2) > THRESHOLD)?joystickExponential(THRESHOLD,maxPower,J1Y2) - RIGHT_OFFSET:0;
        motor[DRIVEBL] = (abs(J1Y1) > THRESHOLD)?joystickExponential(THRESHOLD,maxPower,J1Y1) - LEFT_OFFSET:0;
        motor[DRIVEBR] = (abs(J1Y2) > THRESHOLD)?joystickExponential(THRESHOLD,maxPower,J1Y2) - RIGHT_OFFSET:0;
    }else{
        motor[DRIVEFL] = (abs(J1Y1) > THRESHOLD)?joystickLinear(THRESHOLD,maxPower,J1Y1) - LEFT_OFFSET:0;
        motor[DRIVEFR] = (abs(J1Y2) > THRESHOLD)?joysticLinear(THRESHOLD,maxPower,J1Y2) - RIGHT_OFFSET:0;
        motor[DRIVEBL] = (abs(J1Y1) > THRESHOLD)?joystickLinear(THRESHOLD,maxPower,J1Y1) - LEFT_OFFSET:0;
        motor[DRIVEBR] = (abs(J1Y2) > THRESHOLD)?joystickLinear(THRESHOLD,maxPower,J1Y2) - RIGHT_OFFSET:0;
    }
#elif defined(DRIVER) && defined(DRIVEL) && defined(THRESHOLD)
    if(exponential){
        motor[DRIVEL] = (abs(J1Y1) > THRESHOLD)?joystickExponential(THRESHOLD,maxPower,J1Y1) - LEFT_OFFSET:0;
        motor[DRIVER] = (abs(J1Y2) > THRESHOLD)?joystickExponential(THRESHOLD,maxPower,J1Y2) - RIGHT_OFFSET:0;
    }else{
        motor[DRIVEL] = (abs(J1Y1) > THRESHOLD)?joystickLinear(THRESHOLD,maxPower,J1Y1) - LEFT_OFFSET:0;
        motor[DRIVER] = (abs(J1Y2) > THRESHOLD)?joystickLinear(THRESHOLD,maxPower,J1Y2) - RIGHT_OFFSET:0;
    }
#endif
}
/*
void driveArcade(int maxPower, bool exponential, int joy){
#if defined(__HOLO_H__) && defined(DRIVEFR) && defined(DRIVEFL) && defined(DRIVEBR) && defined(DRIVEBL) && defined(THRESHOLD)
    if(exponential){

    }
#elif defined(DRIVER) && defined(DRIVEL) && defined(THRESHOLD)
#endif
}

#if defined(__HOLO_H__) && defined(DRIVEFR) && defined(DRIVEFL) && defined(DRIVEBR) && defined(DRIVEBL) && defined(THRESHOLD)
void driveHolonomic(int maxPower, bool exponential, int joy){
    if(joy == 1){

    }
}
#endif
*/
#endif //__TELEOP_FUNCS_H__
