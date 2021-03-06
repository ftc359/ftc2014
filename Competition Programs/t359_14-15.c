#pragma config(Hubs,  S1, HTMotor,  HTServo,  HTMotor,  none)
#pragma config(Sensor, S2,     _ir_front,      sensorI2CCustom)
#pragma config(Sensor, S3,     _gyro,          sensorI2CHiTechnicGyro)
#pragma config(Sensor, S4,     _ir_back,       sensorI2CCustom)
#pragma config(Motor,  mtr_S1_C1_1,     leftWheel,     tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     rightWheel,    tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C3_1,     intake,        tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C3_2,     lift,          tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Servo,  srvo_S1_C2_1,    dragger,              tServoStandard)
#pragma config(Servo,  srvo_S1_C2_2,    scorer,               tServoStandard)
#pragma config(Servo,  srvo_S1_C2_3,    front_dragger,        tServoStandard)
#pragma config(Servo,  srvo_S1_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"
#include "Assets\Headers\h359_14-15.h"
#include "Assets\Headers\Teleop_Funcs.h"

int maxDrive = 100;

task main()
{
	initializeRobot();
	waitForStart();
	while(true){
		getJoystickSettings(joystick);
		if(abs(joystick.joy1_y1) >= THRESHOLD)
			motor[leftWheel] = joystickExponential(THRESHOLD, maxDrive-LEFT_OFFSET, joystick.joy1_y1);
		else motor[leftWheel] = 0;
		if(abs(joystick.joy1_y2) >= THRESHOLD)
			motor[rightWheel] = joystickExponential(THRESHOLD, maxDrive-RIGHT_OFFSET, joystick.joy1_y2);
		else motor[rightWheel] = 0;
		if(abs(joystick.joy2_y1) >= THRESHOLD)
			motor[intake] = joystickExponential(THRESHOLD, 100, joystick.joy2_y1);
		else motor[intake] = 0;
		if(abs(joystick.joy2_y2) >= THRESHOLD)
			motor[lift] = joystickExponential(THRESHOLD,100,joystick.joy2_y2);
		else
			motor[lift] = 0;
		if(toggleJ1B(5, true))
			servo[dragger] = DRAGGER_UP;
		if(toggleJ1B(6, true))
			servo[dragger] = DRAGGER_DOWN;
		if(toggleJ1B(7, true))
			servo[front_dragger] = FRONT_DRAGGER_UP;
		if(toggleJ1B(8, true))
			servo[front_dragger] = FRONT_DRAGGER_DOWN;
		if(toggleJ2B(2, true))
			servo[scorer] = SCORER_CLOSE;
		if(toggleJ2B(3, true))
			servo[scorer] = SCORER_OPEN_RG;
		if(toggleJ2B(4, true))
			servo[scorer] = SCORER_OPEN_CG;
	}
}
