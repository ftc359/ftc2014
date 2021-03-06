#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTMotor,  HTMotor)
#pragma config(Motor,  mtr_S1_C1_1,     S1_C1_1,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     S1_C1_2,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     S1_C2_1,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     S1_C2_2,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_1,     S1_C3_1,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C3_2,     S1_C3_2,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     S1_C4_1,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     S1_C4_2,       tmotorTetrix, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*
===Dual Motor Tester===

Made by Shupe
Edited by Ben and David

===Instructions===
	Controller Selection
Use the arrows to change the motor controller.
Press and release the orange button to select that controller.
	Motor Running
Press and release the orange button to change motors.
Press and hold the orange button to run selected motor(s).
Use the arrows to change the speed of the selected motor(s).
*/

int speed[3][4];
int currentMotor[3] = {1,1,1}; //3 = both
int motorController = 1;
bool bValChanged;

int accelerate(int startSpeed, int stopSpeed, long maxTime, long currentTime){
	return ((startSpeed-stopSpeed)/pow(maxTime,2))*pow(currentTime-maxTime,2)+stopSpeed;
}

void motor1(int speed){
	switch(motorController){
		case 1:
			motor[S1_C1_1] = speed;
			break;
		case 2:
			motor[S1_C2_1] = speed;
			break;
		case 3:
			motor[S1_C3_1] = speed;
			break;
		case 4:
			motor[S1_C4_1] = speed;
			break;
	}
}

void motor2(int speed){
	switch(motorController){
		case 1:
			motor[S1_C1_2] = speed;
			break;
		case 2:
			motor[S1_C2_2] = speed;
			break;
		case 3:
			motor[S1_C3_2] = speed;
			break;
		case 4:
			motor[S1_C4_2] = speed;
			break;
	}
}

task controllerFlash{
	while(true){
		clearTimer(T1);
		while(time1(T1) <= 500)
			nxtDisplayTextLine(1,"Controller: [%d]",motorController);
		while(bValChanged)
			nxtDisplayTextLine(1,"Controller: [%d]",motorController);
		clearTimer(T1);
		nxtDisplayTextLine(1,"Controller:");
		while(time1(T1) <= 500 && !bValChanged){}
	}
}

task main()
{
	while(true){
		startTask(controllerFlash);
		while(true){
			if(nNxtButtonPressed == 1){
				bValChanged = true;
				while(nNxtButtonPressed == 1){}
				if(++motorController > 4)
					motorController = 4;
				clearTimer(T2);
			}
			if(nNxtButtonPressed == 2){
				bValChanged = true;
				while(nNxtButtonPressed == 2){}
				if(--motorController < 1)
					motorController = 1;
				clearTimer(T2);
			}
			if(nNxtButtonPressed == 3){
				while(nNxtButtonPressed == 3){}
				nNxtExitClicks += 1;
				break;
			}
			if(bValChanged && time1(T2) > 250)
				bValChanged = false;
		}
		stopTask(controllerFlash);
		nxtDisplayClearTextLine(1);
		nxtDisplayCenteredTextLine(1,"Controller: %d",motorController);
		if(currentMotor[motorController-1] == 3)
			nxtDisplayTextLine(3, "Both Motors: %d", speed[currentMotor[motorController-1]-1][motorController-1]);
		else
			nxtDisplayTextLine(3, "Motor %d: %d", currentMotor[motorController-1], speed[currentMotor[motorController-1]-1][motorController-1]);
		while(true){
			if(nNxtButtonPressed == 0){
				while(nNxtButtonPressed == 0){}
				nxtDisplayClearTextLine(3);
				break;
			}
			if(nNxtButtonPressed == 3){
				clearTimer(T1);
				while(time1(T1) <= 500 && nNxtButtonPressed != -1){}
				if(time1(T1) <= 500){
					if(++currentMotor[motorController-1] > 3)
						currentMotor[motorController-1] = 1;
					if(currentMotor[motorController-1] == 3)
						nxtDisplayTextLine(3, "Both Motors: %d", speed[currentMotor[motorController-1]-1][motorController-1]);
					else
						nxtDisplayTextLine(3, "Motor %d: %d", currentMotor[motorController-1], speed[currentMotor[motorController-1]-1][motorController-1]);
				}else{
					switch(currentMotor[motorController-1]){
					case 1:
						motor1(speed[currentMotor[motorController-1]-1][motorController-1]);
						break;
					case 2:
						motor2(speed[currentMotor[motorController-1]-1][motorController-1]);
						break;
					case 3:
						motor1(speed[currentMotor[motorController-1]-1][motorController-1]);
						motor2(-speed[currentMotor[motorController-1]-1][motorController-1]);
						break;
					}
					nxtDisplayTextLine(5,"Running...");
					while(nNxtButtonPressed == 3){/*Do nothing*/}
					nxtDisplayClearTextLine(5);
					motor1(0);
					motor2(0);
				}
			}
			if(nNxtButtonPressed == 1){
				clearTimer(T3);
				while(nNxtButtonPressed == 1){
					speed[currentMotor[motorController-1]-1][motorController-1] += 5;
					if(speed[currentMotor[motorController-1]-1][motorController-1] > 100)
						speed[currentMotor[motorController-1]-1][motorController-1] = 100;
					wait1Msec(accelerate(250,75,3000,time1(T3)));
					if(currentMotor[motorController-1] == 3)
						nxtDisplayTextLine(3, "Both Motors: %d", speed[currentMotor[motorController-1]-1][motorController-1]);
					else
						nxtDisplayTextLine(3, "Motor %d: %d", currentMotor[motorController-1], speed[currentMotor[motorController-1]-1][motorController-1]);
				}
			}
			if(nNxtButtonPressed == 2){
				clearTimer(T3);
				while(nNxtButtonPressed == 2){
					speed[currentMotor[motorController-1]-1][motorController-1] -= 5;
					if(speed[currentMotor[motorController-1]-1][motorController-1] < -100)
						speed[currentMotor[motorController-1]-1][motorController-1] = -100;
					wait1Msec(accelerate(250,75,3000,time1(T3)));
					if(currentMotor[motorController-1] == 3)
						nxtDisplayTextLine(3, "Both Motors: %d", speed[currentMotor[motorController-1]-1][motorController-1]);
					else
						nxtDisplayTextLine(3, "Motor %d: %d", currentMotor[motorController-1], speed[currentMotor[motorController-1]-1][motorController-1]);
				}
			}

		}
	}
}
