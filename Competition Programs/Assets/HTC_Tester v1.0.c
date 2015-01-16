#pragma config(Sensor, S1,     HTC_1,          sensorI2CCustom)
#pragma config(Sensor, S2,     HTC_2,          sensorI2CCustom)
#pragma config(Sensor, S3,     HTC_3,          sensorI2CCustom)
#pragma config(Sensor, S4,     HTC_4,          sensorI2CCustom)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*
 *
 *Made by FTC Team 359
 *Special thanks to Xander for debugging and common.h and MHTS(Titan Robotics) for ripping off their UI xD
 *
 *Read README.md for instructions
 *
 *Released under the GNU General Public License v3.0
 *
 *Changelog
 *v1.0
 *-Initial release
 */

#include "Headers\HTC.h"

struct mtr{
	int power;
};
struct srvo{
	int val;
};
struct channel{
	int type;
	int selectedLine;
	struct mtr mtrs[2];
	struct srvo srvos[6];
};
struct port{
	struct channel chans[4];
	bool available;
	int channel;
};

struct port ports[4];

tSensors HTC(int channel){
	switch(channel){
	case 1:
		return HTC_1;
	case 2:
		return HTC_2;
	case 3:
		return HTC_3;
	case 4:
		return HTC_4;
	}
	return HTC_1;
}
void runMotor(int port, int channel, int mtr, byte power){
	int index = ((port-1)*8)+((channel-1)*2)+(mtr-1);
  HTMCPower(HTC(port), index, power);
}

void runServo(int port, int channel, int ser, byte val, tSCPWM mode){
	int index = ((port-1)*24)+((channel-1)*6)+(ser-1);
	HTSCServo(HTC(port), index, val, mode);
}

int accelerate(int startSpeed, int stopSpeed, long maxTime, long currentTime){
	return ((startSpeed-stopSpeed)/pow(maxTime,2))*pow(currentTime-maxTime,2)+stopSpeed;
}

void highlightLine(int line){
	for(int index = 7; index > 0; index--)
		invertLine(0, ((7-line)*8)+index, 99, ((7-line)*8)+index);
}

task main()
{
	int iMenuLevel = 0;
	int iPrevMenuLevel = -1;
	int iPort;
	int iMinPort;
	int iMaxPort;
	int iMinChannel;
	int iMaxChannel;
	string sTemp[4];

	displayString(3, "Initializing...");
	for(int tPort = 0; tPort < 4; tPort++){
		for(int chain_index = 0; chain_index < 4; chain_index++){
			ports[tPort].chans[chain_index].type = HTCVerifyType(HTC(tPort+1), chain_index+1);
			displayString(5, "%d/4 %d/4", tPort+1, chain_index+1);
			wait1Msec(10);
		}
		ports[tPort].available = (ports[tPort].chans[0].type > 0 || ports[tPort].chans[1].type > 0 || ports[tPort].chans[2].type > 0 || ports[tPort].chans[3].type > 0);
		ports[tPort].channel = (ports[tPort].chans[0].type > 0)?0:(ports[tPort].chans[1].type > 0)?1:(ports[tPort].chans[2].type > 0)?2:3;
		wait1Msec(50);
	}
	eraseDisplay();
	if(!ports[0].available && !ports[1].available && !ports[2].available && !ports[3].available){
		hogCPU();
		playSound(soundBeepBeep);
		displayCenteredTextLine(3, "No Controllers");
		displayCenteredTextLine(4, "Detected!");
		wait1Msec(2000);
		return;
	}
	displayCenteredTextLine(0, "HTCs Detected!");
	for(int tPort = 0; tPort < 4; tPort++){
		for(int chain_index = 0; chain_index < 4; chain_index++){
		sTemp[chain_index] = (ports[tPort].chans[chain_index].type <= 0)?"NA":(ports[tPort].chans[chain_index].type == 1)?"MC":"SC";
		}
		displayCenteredTextLine(1+(tPort*2), "%s  %s  %s  %s", sTemp[0], sTemp[1], sTemp[2], sTemp[3]);
		if(ports[tPort].available)
			highlightLine(1+(tPort*2));
	}
	wait1Msec(3000);
	eraseDisplay();

	iMinPort = (ports[0].available)?0:(ports[1].available)?1:(ports[2].available)?2:3;
	iMaxPort = (ports[3].available)?3:(ports[2].available)?2:(ports[1].available)?1:0;
	iPort = iMinPort;

	while(true){
		//Port selection
		while(true){
			if(iMenuLevel != 0)
				break;
			if(iMinPort == iMaxPort){ //Only One Port Available
				iMenuLevel = 1;				 //Skip port selection Process
				break;
			}
			if(iPrevMenuLevel != iMenuLevel){
				eraseDisplay();
				sleep(10);
				displayCenteredTextLine(1, "Select A Port");
				displayCenteredTextLine(3, "[S%d]", iPort+1);
				highlightLine(3);
				iPrevMenuLevel = iMenuLevel;
			}
			if(nNxtButtonPressed == 1){
				do{
					if(++iPort > iMaxPort)
						iPort = iMaxPort;
				}while(!ports[iPort].available);
				displayClearTextLine(3);
				displayCenteredTextLine(3, "[S%d]", iPort+1);
				highlightLine(3);
				while(nNxtButtonPressed == 1){}
			}
			if(nNxtButtonPressed == 2){
				do{
					if(--iPort < iMinPort)
						iPort = iMinPort;
				}while(!ports[iPort].available);
				displayClearTextLine(3);
				displayCenteredTextLine(3, "[S%d]", iPort+1);
				highlightLine(3);
				while(nNxtButtonPressed == 2){}
			}
			if(nNxtButtonPressed == 3){
				iMenuLevel = 1;
				nNxtExitClicks = 2;
				while(nNxtButtonPressed == 3){}
				break;
			}
		}
		//Channel Selection
		iMinChannel = (ports[iPort].chans[0].type > 0)?0:(ports[iPort].chans[1].type > 0)?1:(ports[iPort].chans[2].type > 0)?2:3;
		iMaxChannel = (ports[iPort].chans[3].type > 0)?3:(ports[iPort].chans[2].type > 0)?2:(ports[iPort].chans[1].type > 0)?1:0;

		while(true){
			if(iMenuLevel != 1)
				break;
			if(iMinChannel == iMaxChannel){	//Only One Channel Available
				iMenuLevel = 2;
				break;
			}
			if(iPrevMenuLevel != iMenuLevel){
				eraseDisplay();
				sleep(10);
				displayCenteredTextLine(0, "[S%d]", iPort+1);
				displayCenteredTextLine(1, "Select A Channel");
				displayCenteredTextLine(3, "[C%d]", ports[iPort].channel+1);
				highlightLine(3);
				iPrevMenuLevel = iMenuLevel;
			}
			if(nNxtButtonPressed == 0){
				iMenuLevel = 0;
				break;
			}
			if(nNxtButtonPressed == 1){
				do{
					if(++ports[iPort].channel > iMaxChannel)
						ports[iPort].channel = iMaxChannel;
				}while(ports[iPort].chans[ports[iPort].channel].type <= 0);
				displayClearTextLine(3);
				displayCenteredTextLine(3, "[C%d]", ports[iPort].channel+1);
				highlightLine(3);
				while(nNxtButtonPressed == 1){}
			}
			if(nNxtButtonPressed == 2){
				do{
					if(--ports[iPort].channel < iMinChannel)
						ports[iPort].channel = iMinChannel;
				}while(ports[iPort].chans[ports[iPort].channel].type <= 0);
				displayClearTextLine(3);
				displayCenteredTextLine(3, "[C%d]", ports[iPort].channel+1);
				highlightLine(3);
				while(nNxtButtonPressed == 2){}
			}
			if(nNxtButtonPressed == 3){
				iMenuLevel = 2;
				nNxtExitClicks = 2;
				while(nNxtButtonPressed == 3){}
				break;
			}
		}
		//Running
		while(true){
			if(iMenuLevel != 2)
				break;
			if(ports[iPort].chans[ports[iPort].channel].selectedLine == 0)
				ports[iPort].chans[ports[iPort].channel].selectedLine = 2;
			if(iPrevMenuLevel != iMenuLevel){
				eraseDisplay();
				sleep(10);
				displayCenteredTextLine(0, "[S%d][C%d]", iPort+1, ports[iPort].channel+1);
				if(ports[iPort].chans[ports[iPort].channel].type == 1){		//Motor Controller
					displayString(2, "Motor 1: %d", ports[iPort].chans[ports[iPort].channel].mtrs[0].power);
					displayString(4, "Motor 2: %d", ports[iPort].chans[ports[iPort].channel].mtrs[1].power);
					highlightLine(ports[iPort].chans[ports[iPort].channel].selectedLine);
				}else{	//Servo Controller
					for(int index = 0; index <= 5; index++)
						displayString(2+index, "Servo %d: %d", index+1, ports[iPort].chans[ports[iPort].channel].srvos[index].val);
					highlightLine(ports[iPort].chans[ports[iPort].channel].selectedLine);
				}
				iPrevMenuLevel = iMenuLevel;
			}
			if(nNxtButtonPressed == 0){
				iMenuLevel = 1;
				break;
			}
			if(ports[iPort].chans[ports[iPort].channel].type == 1){				//Motor Controller
				if(nNxtButtonPressed == 1){
					clearTimer(T2);
					while(nNxtButtonPressed == 1){
						if(time1[T2] <= 300)
							ports[iPort].chans[ports[iPort].channel].mtrs[(ports[iPort].chans[ports[iPort].channel].selectedLine == 2)?0:1].power++;
						else{
							if(ports[iPort].chans[ports[iPort].channel].mtrs[(ports[iPort].chans[ports[iPort].channel].selectedLine == 2)?0:1].power%5 != 0)
								ports[iPort].chans[ports[iPort].channel].mtrs[(ports[iPort].chans[ports[iPort].channel].selectedLine == 2)?0:1].power += 5-(ports[iPort].chans[ports[iPort].channel].mtrs[(ports[iPort].chans[ports[iPort].channel].selectedLine == 2)?0:1].power%5);
							ports[iPort].chans[ports[iPort].channel].mtrs[(ports[iPort].chans[ports[iPort].channel].selectedLine == 2)?0:1].power += 5;
						}
						if(ports[iPort].chans[ports[iPort].channel].mtrs[(ports[iPort].chans[ports[iPort].channel].selectedLine == 2)?0:1].power > 100)
							ports[iPort].chans[ports[iPort].channel].mtrs[(ports[iPort].chans[ports[iPort].channel].selectedLine == 2)?0:1].power = 100;
						displayClearTextLine(ports[iPort].chans[ports[iPort].channel].selectedLine);
						displayString(ports[iPort].chans[ports[iPort].channel].selectedLine, "Motor %d: %d", ports[iPort].chans[ports[iPort].channel].selectedLine/2, ports[iPort].chans[ports[iPort].channel].mtrs[(ports[iPort].chans[ports[iPort].channel].selectedLine == 2)?0:1].power);
						highlightLine(ports[iPort].chans[ports[iPort].channel].selectedLine);
						if(time1[T2] <= 3000)
							wait1Msec(accelerate(250,75,3000,time1[T2]));
						else
							wait1Msec(75);
					}
				}
				if(nNxtButtonPressed == 2){
					clearTimer(T2);
					while(nNxtButtonPressed == 2){
						if(time1[T2] <= 300)
							ports[iPort].chans[ports[iPort].channel].mtrs[(ports[iPort].chans[ports[iPort].channel].selectedLine == 2)?0:1].power--;
						else{
							if(ports[iPort].chans[ports[iPort].channel].mtrs[(ports[iPort].chans[ports[iPort].channel].selectedLine == 2)?0:1].power%5 != 0)
								ports[iPort].chans[ports[iPort].channel].mtrs[(ports[iPort].chans[ports[iPort].channel].selectedLine == 2)?0:1].power -= (ports[iPort].chans[ports[iPort].channel].mtrs[(ports[iPort].chans[ports[iPort].channel].selectedLine == 2)?0:1].power%5);
							ports[iPort].chans[ports[iPort].channel].mtrs[(ports[iPort].chans[ports[iPort].channel].selectedLine == 2)?0:1].power -= 5;
						}
						if(ports[iPort].chans[ports[iPort].channel].mtrs[(ports[iPort].chans[ports[iPort].channel].selectedLine == 2)?0:1].power < -100)
							ports[iPort].chans[ports[iPort].channel].mtrs[(ports[iPort].chans[ports[iPort].channel].selectedLine == 2)?0:1].power = -100;
						displayClearTextLine(ports[iPort].chans[ports[iPort].channel].selectedLine);
						displayString(ports[iPort].chans[ports[iPort].channel].selectedLine, "Motor %d: %d", ports[iPort].chans[ports[iPort].channel].selectedLine/2, ports[iPort].chans[ports[iPort].channel].mtrs[(ports[iPort].chans[ports[iPort].channel].selectedLine == 2)?0:1].power);
						highlightLine(ports[iPort].chans[ports[iPort].channel].selectedLine);
						if(time1[T2] <= 3000)
							wait1Msec(accelerate(250,75,3000,time1[T2]));
						else
							wait1Msec(75);
					}
				}
				if(nNxtButtonPressed == 3){
					clearTimer(T2);
					while(time1(T2) <= 500 && nNxtButtonPressed == 3){}
					if(time1[T2] <= 500){
						displayClearTextLine(ports[iPort].chans[ports[iPort].channel].selectedLine);
						displayString(ports[iPort].chans[ports[iPort].channel].selectedLine, "Motor %d: %d", ports[iPort].chans[ports[iPort].channel].selectedLine/2, ports[iPort].chans[ports[iPort].channel].mtrs[(ports[iPort].chans[ports[iPort].channel].selectedLine == 2)?0:1].power);
						ports[iPort].chans[ports[iPort].channel].selectedLine += 2;
						if(ports[iPort].chans[ports[iPort].channel].selectedLine > 4)
							ports[iPort].chans[ports[iPort].channel].selectedLine = 2;
						displayClearTextLine(ports[iPort].chans[ports[iPort].channel].selectedLine);
						displayString(ports[iPort].chans[ports[iPort].channel].selectedLine, "Motor %d: %d", ports[iPort].chans[ports[iPort].channel].selectedLine/2, ports[iPort].chans[ports[iPort].channel].mtrs[(ports[iPort].chans[ports[iPort].channel].selectedLine == 2)?0:1].power);
						highlightLine(ports[iPort].chans[ports[iPort].channel].selectedLine);
						while(nNxtButtonPressed == 3){}
					}else{
						displayString(6, "Running...");
						while(nNxtButtonPressed == 3)
							runMotor(iPort+1, ports[iPort].channel+1, (ports[iPort].chans[ports[iPort].channel].selectedLine == 2)?1:2, ports[iPort].chans[ports[iPort].channel].mtrs[(ports[iPort].chans[ports[iPort].channel].selectedLine == 2)?0:1].power);
						runMotor(iPort+1, ports[iPort].channel+1, (ports[iPort].chans[ports[iPort].channel].selectedLine == 2)?1:2, 0);
						displayClearTextLine(6);
					}
				}
			}else if(ports[iPort].chans[ports[iPort].channel].type == 2){	//Servo Controller
					if(nNxtButtonPressed == 1){
					clearTimer(T2);
					while(nNxtButtonPressed == 1){
						if(time1[T2] <= 300)
							ports[iPort].chans[ports[iPort].channel].srvos[ports[iPort].chans[ports[iPort].channel].selectedLine-2].val++;
						else{
							if(ports[iPort].chans[ports[iPort].channel].srvos[ports[iPort].chans[ports[iPort].channel].selectedLine-2].val%5 != 0)
								ports[iPort].chans[ports[iPort].channel].srvos[ports[iPort].chans[ports[iPort].channel].selectedLine-2].val += 5-(ports[iPort].chans[ports[iPort].channel].srvos[ports[iPort].chans[ports[iPort].channel].selectedLine-2].val%5);
							ports[iPort].chans[ports[iPort].channel].srvos[ports[iPort].chans[ports[iPort].channel].selectedLine-2].val += 5;
						}
						if(ports[iPort].chans[ports[iPort].channel].srvos[ports[iPort].chans[ports[iPort].channel].selectedLine-2].val > 255)
							ports[iPort].chans[ports[iPort].channel].srvos[ports[iPort].chans[ports[iPort].channel].selectedLine-2].val = 255;
						displayClearTextLine(ports[iPort].chans[ports[iPort].channel].selectedLine);
						displayString(ports[iPort].chans[ports[iPort].channel].selectedLine, "Servo %d: %d", ports[iPort].chans[ports[iPort].channel].selectedLine-1, ports[iPort].chans[ports[iPort].channel].srvos[ports[iPort].chans[ports[iPort].channel].selectedLine-2]);
						highlightLine(ports[iPort].chans[ports[iPort].channel].selectedLine);
						if(time1[T2] <= 3500)
							wait1Msec(accelerate(300,75,3500,time1[T2]));
						else
							wait1Msec(75);
					}
				}
				if(nNxtButtonPressed == 2){
					clearTimer(T2);
					while(nNxtButtonPressed == 2){
						if(time1[T2] <= 300)
							ports[iPort].chans[ports[iPort].channel].srvos[ports[iPort].chans[ports[iPort].channel].selectedLine-2].val--;
						else{
							if(ports[iPort].chans[ports[iPort].channel].srvos[ports[iPort].chans[ports[iPort].channel].selectedLine-2].val%5 != 0)
								ports[iPort].chans[ports[iPort].channel].srvos[ports[iPort].chans[ports[iPort].channel].selectedLine-2].val -= (ports[iPort].chans[ports[iPort].channel].srvos[ports[iPort].chans[ports[iPort].channel].selectedLine-2].val%5);
							ports[iPort].chans[ports[iPort].channel].srvos[ports[iPort].chans[ports[iPort].channel].selectedLine-2].val -= 5;
						}
						if(ports[iPort].chans[ports[iPort].channel].srvos[ports[iPort].chans[ports[iPort].channel].selectedLine-2].val < 0)
							ports[iPort].chans[ports[iPort].channel].srvos[ports[iPort].chans[ports[iPort].channel].selectedLine-2].val = 0;
						displayClearTextLine(ports[iPort].chans[ports[iPort].channel].selectedLine);
						displayString(ports[iPort].chans[ports[iPort].channel].selectedLine, "Servo %d: %d", ports[iPort].chans[ports[iPort].channel].selectedLine-1, ports[iPort].chans[ports[iPort].channel].srvos[ports[iPort].chans[ports[iPort].channel].selectedLine-2].val);
						highlightLine(ports[iPort].chans[ports[iPort].channel].selectedLine);
						if(time1[T2] <= 3500)
							wait1Msec(accelerate(300,75,3500,time1[T2]));
						else
							wait1Msec(75);
					}
				}
				if(nNxtButtonPressed == 3){
					clearTimer(T2);
					while(time1(T2) <= 500 && nNxtButtonPressed == 3){}
					if(time1[T2] <= 500){
						displayClearTextLine(ports[iPort].chans[ports[iPort].channel].selectedLine);
						displayString(ports[iPort].chans[ports[iPort].channel].selectedLine, "Servo %d: %d", ports[iPort].chans[ports[iPort].channel].selectedLine-1, ports[iPort].chans[ports[iPort].channel].srvos[ports[iPort].chans[ports[iPort].channel].selectedLine-2]);
						if(++ports[iPort].chans[ports[iPort].channel].selectedLine > 7)
							ports[iPort].chans[ports[iPort].channel].selectedLine = 2;
						displayClearTextLine(ports[iPort].chans[ports[iPort].channel].selectedLine);
						displayString(ports[iPort].chans[ports[iPort].channel].selectedLine, "Servo %d: %d", ports[iPort].chans[ports[iPort].channel].selectedLine-1, ports[iPort].chans[ports[iPort].channel].srvos[ports[iPort].chans[ports[iPort].channel].selectedLine-2]);
						highlightLine(ports[iPort].chans[ports[iPort].channel].selectedLine);
						while(nNxtButtonPressed == 3){}
					}else{
						displayString(1, "Running...");
						runServo(iPort+1, ports[iPort].channel+1, ports[iPort].chans[ports[iPort].channel].selectedLine-1, ports[iPort].chans[ports[iPort].channel].srvos[ports[iPort].chans[ports[iPort].channel].selectedLine-2].val, disable_timeout);
						while(nNxtButtonPressed == 3){}
						runServo(iPort+1, ports[iPort].channel+1, ports[iPort].chans[ports[iPort].channel].selectedLine-1, ports[iPort].chans[ports[iPort].channel].srvos[ports[iPort].chans[ports[iPort].channel].selectedLine-2].val, force_timeout);
						displayClearTextLine(1);
					}
				}
			}
		}
	}
}