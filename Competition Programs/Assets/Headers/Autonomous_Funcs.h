#ifndef __AUTONOMOUS_FUNCS_H__
#define __AUTONOMOUS_FUNCS_H__

#pragma systemFile

#ifndef RIGHT_OFFSET
#define RIGHT_OFFSET	0
#endif

#ifndef LEFT_OFFSET
#define LEFT_OFFSET	0
#endif

#ifndef  ENC_ERRORMARGIN       //To prevent encoder errors.
#define  ENC_ERRORMARGIN   2000
#endif

#ifndef MAX_LINES
#define MAX_LINES       8
#endif

#define CONFIG_UNSET    0
#define CONFIG_CONSTANT 1
#define CONFIG_BYTE     2
#define CONFIG_SHORT    3
#define CONFIG_LONG     4
#define CONFIG_BOOL     5
#define CONFIG_STRING   6
#define CONFIG_CHAR     7

enum tDirection{
    fwd = 0,
    left = 1,
    right = 2,
    bwd = 3
};

struct nxtDisplayLineData{
    unsigned byte config;
    string desc;
    string units;
    void *ptr;
    long lowerLimit;
    long increment;
    long upperLimit;
    unsigned byte arraySize;
    string valTrue;
    string valFalse;
};

struct nxtDisplayLineData nxtDisplayLines[MAX_LINES];

int iLine;

//Function prototypes
void move(int power, long time, tDirection dir);
void moveEnc(int power, long distance, tDirection dir);
void moveEncSingle(tMotor mtr, int power, long distance);
int accelerate(int startSpeed, int stopSpeed, long maxTime, long currentTime);

void configLine(char* str);
void configLine(char* desc, sbyte *ptr, char* units, sbyte lowerLimit, sbyte increment, sbyte upperLimit);
void configLine(char* desc, short *ptr, char* units, short lowerLimit, short increment, short upperLimit);
void configLine(char* desc, long *ptr, char* units, long lowerLimit, long increment, long upperLimit);	//Why no int? Byte size of int = long
void configLine(char* desc, bool *ptr, char* units, char* valTrue, char* valFalse);
//void configLine(char* desc, string *ptr, char* units, ubyte arraySize);
//void configLine(char* desc, char *ptr, char* units, ubyte arraySize);
void startDisplay(bool onPress, bool savePrefs);

void move(int power, long time, tDirection dir){
#if defined(__HOLO_H__) && defined(DRIVEFR) && defined(DRIVEFL) && defined(DRIVEBR) && defined(DRIVEBL)
    motor[DRIVEFL] = (power - LEFT_OFFSET)*(dir & 1)?-1:1;
    motor[DRIVEFR] = (power - RIGHT_OFFSET)*(dir & 2)?-1:1;
    motor[DRIVEBL] = (power - LEFT_OFFSET)*(dir & 1)?-1:1;
    motor[DRIVEBR] = (power - RIGHT_OFFSET)*(dir & 2)?-1:1;
    wait1Msec(time);
    motor[DRIVEFL] = 0;
    motor[DRIVEFR] = 0;
    motor[DRIVEBL] = 0;
    motor[DRIVEBR] = 0;
#elif defined(DRIVER) && defined(DRIVEL)
    motor[DRIVEL] = (power - LEFT_OFFSET)*((dir & 1)?-1:1);
    motor[DRIVER] = (power - RIGHT_OFFSET)*((dir & 2)?-1:1);
    wait1Msec(time);
    motor[DRIVEL] = 0;
    motor[DRIVER] = 0;
#endif
}
void moveEnc(int power, long distance, tDirection dir){
    long nPrev[4];
#if defined(__HOLO_H__) && defined(DRIVEFR) && defined(DRIVEFL) && defined(DRIVEBR) && defined(DRIVEBL)
    nMotorEncoder[DRIVEFL] = 0;
    nMotorEncoder[DRIVEFR] = 0;
    nMotorEncoder[DRIVEBL] = 0;
    nMotorEncoder[DRIVEBR] = 0;
    wait1Msec(300); //wait for the encoders to calibrate
    motor[DRIVEFL] = (power - LEFT_OFFSET)*(dir & 1)?-1:1;
    motor[DRIVEFR] = (power - RIGHT_OFFSET)*(dir & 2)?-1:1;
    motor[DRIVEBL] = (power - LEFT_OFFSET)*(dir & 1)?-1:1;
    motor[DRIVEBR] = (power - RIGHT_OFFSET)*(dir & 2)?-1:1;
    while(nPrev[0] < distance && nPrev[1] < distance && nPrev[2] < distance && nPrev[3] < distance){
        nPrev[0] = (abs(nMotorEncoder[DRIVEFL]) >= nPrev[0] +  ENC_ERRORMARGIN)?nPrev[0]:abs(nMotorEncoder[DRIVEFL]);
        nPrev[1] = (abs(nMotorEncoder[DRIVEFR]) >= nPrev[1] +  ENC_ERRORMARGIN)?nPrev[1]:abs(nMotorEncoder[DRIVEFR]);
        nPrev[2] = (abs(nMotorEncoder[DRIVEBL]) >= nPrev[2] +  ENC_ERRORMARGIN)?nPrev[2]:abs(nMotorEncoder[DRIVEBL]);
        nPrev[3] = (abs(nMotorEncoder[DRIVEBR]) >= nPrev[3] +  ENC_ERRORMARGIN)?nPrev[3]:abs(nMotorEncoder[DRIVEBR]);
    }
    motor[DRIVEFL] = 0;
    motor[DRIVEFR] = 0;
    motor[DRIVEBL] = 0;
    motor[DRIVEBR] = 0;
#elif defined(DRIVER) && defined(DRIVEL)
    nMotorEncoder[DRIVEL] = 0;
    nMotorEncoder[DRIVER] = 0;
    wait1Msec(300); //wait for the encoders to calibrate
    motor[DRIVEL] = (power - LEFT_OFFSET)*(dir & 1)?-1:1;
    motor[DRIVER] = (power - RIGHT_OFFSET)*(dir & 2)?-1:1;
    while(nPrev[0] < distance && nPrev[1] < distance){
        nPrev[0] = (abs(nMotorEncoder[DRIVEL]) >= nPrev[0] +  ENC_ERRORMARGIN)?nPrev[0]:abs(nMotorEncoder[DRIVEL]);
        nPrev[1] = (abs(nMotorEncoder[DRIVER]) >= nPrev[1] +  ENC_ERRORMARGIN)?nPrev[1]:abs(nMotorEncoder[DRIVER]);
    }
    motor[DRIVEL] = 0;
    motor[DRIVER] = 0;
#endif
}
void moveEncSingle(tMotor mtr, int power, long distance){
	long nPrev;
	nMotorEncoder[mtr] = 0;
	wait1Msec(300);	//wait for the encoders to calibrate
	motor[mtr] = power;
	while(nPrev < distance)
		nPrev = (abs(nMotorEncoder[mtr]) >= nPrev + ENC_ERRORMARGIN)?nPrev:abs(nMotorEncoder[mtr]);
	motor[mtr] = 0;
}

int accelerate(int startSpeed, int stopSpeed, long maxTime, long currentTime){
	if(currentTime > maxTime)
		return stopSpeed;
	return ((startSpeed-stopSpeed)/pow(maxTime,2))*pow(currentTime-maxTime,2)+stopSpeed;
}

void configLine(char* str){
    nxtDisplayLines[iLine].config = CONFIG_CONSTANT;
    nxtDisplayLines[iLine++].desc = str;
}
void configLine(char* desc, sbyte *ptr, char* units, sbyte lowerLimit, sbyte increment, sbyte upperLimit){
    nxtDisplayLines[iLine].config = CONFIG_BYTE;
    nxtDisplayLines[iLine].desc = desc;
    nxtDisplayLines[iLine].units = units;
    nxtDisplayLines[iLine].ptr = (void*)(sbyte*)ptr;
    nxtDisplayLines[iLine].lowerLimit = lowerLimit;
    nxtDisplayLines[iLine].increment = increment;
    nxtDisplayLines[iLine++].upperLimit = upperLimit;
}
void configLine(char* desc, short *ptr, char* units, short lowerLimit, short increment, short upperLimit){
    nxtDisplayLines[iLine].config = CONFIG_SHORT;
    nxtDisplayLines[iLine].desc = desc;
    nxtDisplayLines[iLine].units = units;
    nxtDisplayLines[iLine].ptr = (void*)(short*)ptr;
    nxtDisplayLines[iLine].lowerLimit = lowerLimit;
    nxtDisplayLines[iLine].increment = increment;
    nxtDisplayLines[iLine++].upperLimit = upperLimit;
}
void configLine(char* desc, long *ptr, char* units, long lowerLimit, long increment, long upperLimit){
    nxtDisplayLines[iLine].config = CONFIG_LONG;
    nxtDisplayLines[iLine].desc = desc;
    nxtDisplayLines[iLine].units = units;
    nxtDisplayLines[iLine].ptr = (void*)(long*)ptr;
    nxtDisplayLines[iLine].lowerLimit = lowerLimit;
    nxtDisplayLines[iLine].increment = increment;
    nxtDisplayLines[iLine++].upperLimit = upperLimit;
}
void configLine(char* desc, bool *ptr, char* units, char* valTrue, char* valFalse){
    nxtDisplayLines[iLine].config = CONFIG_BOOL;
    nxtDisplayLines[iLine].desc = desc;
    nxtDisplayLines[iLine].units = units;
    nxtDisplayLines[iLine].ptr = (void*)(bool*)ptr;
    nxtDisplayLines[iLine].valTrue = valTrue;
    nxtDisplayLines[iLine++].valFalse = valFalse;
}/*
void configLine(char* desc, string *ptr, char* units, ubyte arraySize){
    nxtDisplayLines[iLine].config = CONFIG_STRING;
    nxtDisplayLines[iLine].desc = desc;
    nxtDisplayLines[iLine].units = units;
    nxtDisplayLines[iLine].ptr = (void*)(string*)ptr;
    nxtDisplayLines[iLine++].arraySize = arraySize;
}
void configLine(char* desc, char *ptr, char* units, ubyte arraySize){
    nxtDisplayLines[iLine++].config = CONFIG_CHAR;
    nxtDisplayLines[iLine].desc = desc;
    nxtDisplayLines[iLine].units = units;
    nxtDisplayLines[iLine++].ptr = (void*)(char*)ptr;
    nxtDisplayLines[iLine++].arraySize = arraySize;
}*/
void startDisplay(bool onPress, bool savePrefs){

    int iNumLines = iLine;
    int iCurrentLine = 0;
    int iOffset = 0;
    ubyte iArrayIndex[MAX_LINES];
    string sTemp;

    TFileHandle     hFileHandle;
    TFileIOResult   nIoResult;
    string kConfigFileName = "DisplayConfig.txt";
    short nFileSize;
    bool bDelete = false;
    byte iTemp;

    if(iNumLines == 0)
        return;

    nNxtExitClicks = 2;

#ifdef getJoystickSettings  //Is JoystickDriver.c included?
    stopTask(displayDiagnostics);
#endif

    eraseDisplay();
    sleep(1);
    eraseDisplay(); //In case screen glitches and does not erase everything the first time

    if(savePrefs){
        nxtDisplayCenteredTextLine(3, "Delete Last Save");
        nxtDisplayCenteredTextLine(4, "No");
        while(true){
        	if(nNxtButtonPressed == 3){
        		if(!onPress)
        			while(nNxtButtonPressed == 3){}
        		break;
        	}
        	if(nNxtButtonPressed == 1){
        		if(!onPress)
        			while(nNxtButtonPressed == 1){}
        		bDelete = !bDelete;
        		displayClearTextLine(4);
        		nxtDisplayCenteredTextLine(4, "%s", (bDelete)?"Yes":"No");
        		if(onPress)
        			while(nNxtButtonPressed == 1){}
        	}
        	if(nNxtButtonPressed == 1){
        		if(!onPress)
        			while(nNxtButtonPressed == 1){}
        		bDelete = !bDelete;
        		displayClearTextLine(4);
        		nxtDisplayCenteredTextLine(4, "%s", (bDelete)?"Yes":"No");
        		if(onPress)
        			while(nNxtButtonPressed == 1){}
        	}
        }
        displayClearTextLine(3);
        displayClearTextLine(4);
        if(bDelete)
       	 Delete(kConfigFileName, nIoResult);
        OpenRead(hFileHandle, nIoResult, kConfigFileName, nFileSize);
        if(nIoResult == ioRsltSuccess){
	        for(int index = 0; index <= iNumLines; index++){
	        	switch(nxtDisplayLines[index].config) {
	     	     case CONFIG_UNSET:
	   		       break;
		          case CONFIG_CONSTANT:
			          break;
		         case CONFIG_BYTE:
			          ReadByte(hFileHandle, nIoResult, *(sbyte*)nxtDisplayLines[index].ptr);
			          break;
		          case CONFIG_SHORT:
			          ReadShort(hFileHandle, nIoResult, *(short*)nxtDisplayLines[index].ptr);
			          break;
  	          case CONFIG_LONG:
			          ReadLong(hFileHandle, nIoResult, *(long*)nxtDisplayLines[index].ptr);
			          break;
			         case CONFIG_BOOL:
			         	ReadByte(hFileHandle, nIoResult, iTemp);
			         	*(bool*)nxtDisplayLines[index].ptr = (iTemp != 0);
			         	break;
		          default:
	 		          break;
	      	  }
	        }
      	}
        Close(hFileHandle, nIoResult);
    }

    for(int index = 0; index < ((iNumLines < 8)?iNumLines:8); index++){
        switch(nxtDisplayLines[index].config){
            case CONFIG_UNSET:
                break;
            case CONFIG_CONSTANT:
                displayString(index, "%s", nxtDisplayLines[index].desc);
                break;
            case CONFIG_BYTE:
                displayString(index, "%s%d%s", nxtDisplayLines[index].desc, *(sbyte*)nxtDisplayLines[index].ptr, nxtDisplayLines[index].units);
                break;
            case CONFIG_SHORT:
                displayString(index, "%s%d%s", nxtDisplayLines[index].desc, *(short*)nxtDisplayLines[index].ptr, nxtDisplayLines[index].units);
                break;
            case CONFIG_LONG:
                displayString(index, "%s%d%s", nxtDisplayLines[index].desc, *(long*)nxtDisplayLines[index].ptr, nxtDisplayLines[index].units);
                break;
            case CONFIG_BOOL:
                sTemp = (*(bool*)nxtDisplayLines[index].ptr)?nxtDisplayLines[index].valTrue:nxtDisplayLines[index].valFalse;
                displayString(index, "%s%s%s", nxtDisplayLines[index].desc, sTemp, nxtDisplayLines[index].units);
                break;
            case CONFIG_STRING:
                displayString(index, "%s%s%s", nxtDisplayLines[index].desc, *(string*)nxtDisplayLines[index].ptr, nxtDisplayLines[index].units);
                break;
            case CONFIG_CHAR:
                displayString(index, "%s%c%s", nxtDisplayLines[index].desc, *(char*)nxtDisplayLines[index].ptr, nxtDisplayLines[index].units);
                break;
        }
    }

    while(nxtDisplayLines[iCurrentLine].config == CONFIG_CONSTANT)
        iCurrentLine++;
    for(int index = 7; index > 0; index--)
        invertLine(0, ((7-iCurrentLine)*8)+index, 99, ((7-iCurrentLine)*8)+index);

    while(true){
        if(nNxtButtonPressed == 0){
            if(!onPress)
                while(nNxtButtonPressed == 0){}
            displayClearTextLine(iCurrentLine);
            switch(nxtDisplayLines[iCurrentLine].config){
                case CONFIG_UNSET:
                    break;
                case CONFIG_CONSTANT:
                    displayString(iCurrentLine-iOffset, "%s", nxtDisplayLines[iCurrentLine].desc);
                    break;
                case CONFIG_BYTE:
                    displayString(iCurrentLine-iOffset, "%s%d%s", nxtDisplayLines[iCurrentLine].desc, *(sbyte*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
                    break;
                case CONFIG_SHORT:
                    displayString(iCurrentLine-iOffset, "%s%d%s", nxtDisplayLines[iCurrentLine].desc, *(short*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
                    break;
                case CONFIG_LONG:
                    displayString(iCurrentLine-iOffset, "%s%d%s", nxtDisplayLines[iCurrentLine].desc, *(long*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
                    break;
                case CONFIG_BOOL:
                    sTemp = (*(bool*)nxtDisplayLines[iCurrentLine].ptr)?nxtDisplayLines[iCurrentLine].valTrue:nxtDisplayLines[iCurrentLine].valFalse;
                    displayString(iCurrentLine-iOffset, "%s%s%s", nxtDisplayLines[iCurrentLine].desc, sTemp, nxtDisplayLines[iCurrentLine].units);
                    break;
                case CONFIG_STRING:
                    displayString(iCurrentLine-iOffset, "%s%s%s", nxtDisplayLines[iCurrentLine].desc, *(string*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
                    break;
                case CONFIG_CHAR:
                    displayString(iCurrentLine-iOffset, "%s%c%s", nxtDisplayLines[iCurrentLine].desc, *(char*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
                    break;
            }
            do{
                if(--iCurrentLine < 0)
                    iCurrentLine = iNumLines;
            }while(nxtDisplayLines[iCurrentLine].config <= 1);    //Constant or unset

            if(false /*iCurrentLine >= 5 && iNumLines > 7*/){
                while(iCurrentLine - iOffset >= 5 && iNumLines - iCurrentLine > 2){
                    iOffset--;
                }
                for(int index = 0; index <= 7; index++){
                    switch(nxtDisplayLines[index].config){
                        case CONFIG_UNSET:
                            break;
                        case CONFIG_CONSTANT:
                            displayString(index, "%s", nxtDisplayLines[index].desc);
                            break;
                        case CONFIG_BYTE:
                            displayString(index, "%s%d%s", nxtDisplayLines[index].desc, *(sbyte*)nxtDisplayLines[index].ptr, nxtDisplayLines[index].units);
                            break;
                        case CONFIG_SHORT:
                            displayString(index, "%s%d%s", nxtDisplayLines[index].desc, *(short*)nxtDisplayLines[index].ptr, nxtDisplayLines[index].units);
                            break;
                        case CONFIG_LONG:
                            displayString(index, "%s%d%s", nxtDisplayLines[index].desc, *(long*)nxtDisplayLines[index].ptr, nxtDisplayLines[index].units);
                            break;
                        case CONFIG_BOOL:
                            sTemp = (*(bool*)nxtDisplayLines[index].ptr)?nxtDisplayLines[index].valTrue:nxtDisplayLines[index].valFalse;
                            displayString(index, "%s%s%s", nxtDisplayLines[index].desc, sTemp, nxtDisplayLines[index].units);
                            break;
                        case CONFIG_STRING:
                            displayString(index, "%s%s%s", nxtDisplayLines[index].desc, *(string*)nxtDisplayLines[index].ptr, nxtDisplayLines[index].units);
                            break;
                        case CONFIG_CHAR:
                            displayString(index, "%s%c%s", nxtDisplayLines[index].desc, *(char*)nxtDisplayLines[index].ptr, nxtDisplayLines[index].units);
                            break;
                    }
                }
            }else{
            	displayClearTextLine(iCurrentLine);
	            switch(nxtDisplayLines[iCurrentLine].config){
	                case CONFIG_UNSET:
	                    break;
	                case CONFIG_CONSTANT:
	                    displayString(iCurrentLine-iOffset, "%s", nxtDisplayLines[iCurrentLine].desc);
	                    break;
	                case CONFIG_BYTE:
	                    displayString(iCurrentLine-iOffset, "%s%d%s", nxtDisplayLines[iCurrentLine].desc, *(sbyte*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
	                    break;
	                case CONFIG_SHORT:
	                    displayString(iCurrentLine-iOffset, "%s%d%s", nxtDisplayLines[iCurrentLine].desc, *(short*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
	                    break;
	                case CONFIG_LONG:
	                    displayString(iCurrentLine-iOffset, "%s%d%s", nxtDisplayLines[iCurrentLine].desc, *(long*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
	                    break;
	                case CONFIG_BOOL:
	                    sTemp = (*(bool*)nxtDisplayLines[iCurrentLine].ptr)?nxtDisplayLines[iCurrentLine].valTrue:nxtDisplayLines[iCurrentLine].valFalse;
	                    displayString(iCurrentLine-iOffset, "%s%s%s", nxtDisplayLines[iCurrentLine].desc, sTemp, nxtDisplayLines[iCurrentLine].units);
	                    break;
	                case CONFIG_STRING:
	                    displayString(iCurrentLine-iOffset, "%s%s%s", nxtDisplayLines[iCurrentLine].desc, *(string*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
	                    break;
	                case CONFIG_CHAR:
	                    displayString(iCurrentLine-iOffset, "%s%c%s", nxtDisplayLines[iCurrentLine].desc, *(char*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
	                    break;
	            }
	            for(int index = 7; index > 0; index--)
        				invertLine(0, ((7-iCurrentLine)*8)+index, 99, ((7-iCurrentLine)*8)+index);
	          }

            if(onPress)
                while(nNxtButtonPressed == 0){}

            nNxtExitClicks += 1;
        }	//if(nNxtButtonPressed == 0)
        if(nNxtButtonPressed == 1){
            if(nxtDisplayLines[iCurrentLine].config == CONFIG_BOOL){
                if(!onPress)
                    while(nNxtButtonPressed == 1){}
                *(bool*)nxtDisplayLines[iCurrentLine].ptr = !*(bool*)nxtDisplayLines[iCurrentLine].ptr;
                displayClearTextLine(iCurrentLine);
                sTemp = (*(bool*)nxtDisplayLines[iCurrentLine].ptr)?nxtDisplayLines[iCurrentLine].valTrue:nxtDisplayLines[iCurrentLine].valFalse;
				        displayString(iCurrentLine-iOffset, "%s%s%s", nxtDisplayLines[iCurrentLine].desc, sTemp, nxtDisplayLines[iCurrentLine].units);
				        for(int index = 7; index > 0; index--)
        					invertLine(0, ((7-iCurrentLine)*8)+index, 99, ((7-iCurrentLine)*8)+index);
                if(onPress)
                    while(nNxtButtonPressed == 1){}
            }else{
                clearTimer(T1);
                while(nNxtButtonPressed == 1){
                    switch(nxtDisplayLines[iCurrentLine].config){
                        case CONFIG_BYTE:
                            *(sbyte*)nxtDisplayLines[iCurrentLine].ptr += nxtDisplayLines[iCurrentLine].increment;
                            if(*(sbyte*)nxtDisplayLines[iCurrentLine].ptr > nxtDisplayLines[iCurrentLine].upperLimit)
                                *(sbyte*)nxtDisplayLines[iCurrentLine].ptr = nxtDisplayLines[iCurrentLine].upperLimit;
                            break;
                        case CONFIG_SHORT:
                            *(short*)nxtDisplayLines[iCurrentLine].ptr += nxtDisplayLines[iCurrentLine].increment;
                            if(*(short*)nxtDisplayLines[iCurrentLine].ptr > nxtDisplayLines[iCurrentLine].upperLimit)
                                *(short*)nxtDisplayLines[iCurrentLine].ptr = nxtDisplayLines[iCurrentLine].upperLimit;
                            break;
                        case CONFIG_LONG:
                            *(long*)nxtDisplayLines[iCurrentLine].ptr += nxtDisplayLines[iCurrentLine].increment;
                            if(*(long*)nxtDisplayLines[iCurrentLine].ptr > nxtDisplayLines[iCurrentLine].upperLimit)
                                *(long*)nxtDisplayLines[iCurrentLine].ptr = nxtDisplayLines[iCurrentLine].upperLimit;
                            break;
                        case CONFIG_STRING:
                            if(++iArrayIndex[iCurrentLine] > nxtDisplayLines[iCurrentLine].arraySize)
                                iArrayIndex[iCurrentLine] = nxtDisplayLines[iCurrentLine].arraySize;
                            *(string*)nxtDisplayLines[iCurrentLine].ptr = *(string*)(nxtDisplayLines[iCurrentLine].ptr + iArrayIndex[iCurrentLine]);
                            break;
                        case CONFIG_CHAR:
                            if(++iArrayIndex[iCurrentLine] > nxtDisplayLines[iCurrentLine].arraySize)
                                iArrayIndex[iCurrentLine] = nxtDisplayLines[iCurrentLine].arraySize;
                            *(char*)nxtDisplayLines[iCurrentLine].ptr = *(char*)(nxtDisplayLines[iCurrentLine].ptr + iArrayIndex[iCurrentLine]);
                            break;
                    }
			              displayClearTextLine(iCurrentLine);
				            switch(nxtDisplayLines[iCurrentLine].config){
				                case CONFIG_UNSET:
				                    break;
				                case CONFIG_CONSTANT:
				                    displayString(iCurrentLine-iOffset, "%s", nxtDisplayLines[iCurrentLine].desc);
				                    break;
				                case CONFIG_BYTE:
				                    displayString(iCurrentLine-iOffset, "%s%d%s", nxtDisplayLines[iCurrentLine].desc, *(sbyte*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
				                    break;
				                case CONFIG_SHORT:
				                    displayString(iCurrentLine-iOffset, "%s%d%s", nxtDisplayLines[iCurrentLine].desc, *(short*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
				                    break;
				                case CONFIG_LONG:
				                    displayString(iCurrentLine-iOffset, "%s%d%s", nxtDisplayLines[iCurrentLine].desc, *(long*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
				                    break;
				                case CONFIG_STRING:
				                    displayString(iCurrentLine-iOffset, "%s%s%s", nxtDisplayLines[iCurrentLine].desc, *(string*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
				                    break;
				                case CONFIG_CHAR:
				                    displayString(iCurrentLine-iOffset, "%s%c%s", nxtDisplayLines[iCurrentLine].desc, *(char*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
				                    break;
				            }
				            for(int index = 7; index > 0; index--)
			        				invertLine(0, ((7-iCurrentLine)*8)+index, 99, ((7-iCurrentLine)*8)+index);
                    wait1Msec(accelerate(300, 50, 5000, time1[T1]));
                }
            }
        }   //if(nNxtButtonPressed == 1)
        if(nNxtButtonPressed == 2){
            if(nxtDisplayLines[iCurrentLine].config == CONFIG_BOOL){
                if(!onPress)
                    while(nNxtButtonPressed == 2){}
                *(bool*)nxtDisplayLines[iCurrentLine].ptr = !*(bool*)nxtDisplayLines[iCurrentLine].ptr;
                displayClearTextLine(iCurrentLine);
                sTemp = (*(bool*)nxtDisplayLines[iCurrentLine].ptr)?nxtDisplayLines[iCurrentLine].valTrue:nxtDisplayLines[iCurrentLine].valFalse;
				        displayString(iCurrentLine-iOffset, "%s%s%s", nxtDisplayLines[iCurrentLine].desc, sTemp, nxtDisplayLines[iCurrentLine].units);
				     		for(int index = 7; index > 0; index--)
        					invertLine(0, ((7-iCurrentLine)*8)+index, 99, ((7-iCurrentLine)*8)+index);
                if(onPress)
                    while(nNxtButtonPressed == 2){}
            }else{
                clearTimer(T1);
                while(nNxtButtonPressed == 2){
                    switch(nxtDisplayLines[iCurrentLine].config){
                        case CONFIG_BYTE:
                            *(sbyte*)nxtDisplayLines[iCurrentLine].ptr -= nxtDisplayLines[iCurrentLine].increment;
                            if(*(sbyte*)nxtDisplayLines[iCurrentLine].ptr < nxtDisplayLines[iCurrentLine].lowerLimit)
                                *(sbyte*)nxtDisplayLines[iCurrentLine].ptr = nxtDisplayLines[iCurrentLine].lowerLimit;
                            break;
                        case CONFIG_SHORT:
                            *(short*)nxtDisplayLines[iCurrentLine].ptr -= nxtDisplayLines[iCurrentLine].increment;
                            if(*(short*)nxtDisplayLines[iCurrentLine].ptr < nxtDisplayLines[iCurrentLine].lowerLimit)
                                *(short*)nxtDisplayLines[iCurrentLine].ptr = nxtDisplayLines[iCurrentLine].lowerLimit;
                            break;
                        case CONFIG_LONG:
                            *(long*)nxtDisplayLines[iCurrentLine].ptr -= nxtDisplayLines[iCurrentLine].increment;
                            if(*(long*)nxtDisplayLines[iCurrentLine].ptr < nxtDisplayLines[iCurrentLine].lowerLimit)
                                *(long*)nxtDisplayLines[iCurrentLine].ptr = nxtDisplayLines[iCurrentLine].lowerLimit;
                            break;
                        case CONFIG_STRING:
                            if(--iArrayIndex[iCurrentLine] < 0)
                                iArrayIndex[iCurrentLine] = 0;
                            *(string*)nxtDisplayLines[iCurrentLine].ptr = *(string*)(nxtDisplayLines[iCurrentLine].ptr + iArrayIndex[iCurrentLine]);
                            break;
                        case CONFIG_CHAR:
                            if(--iArrayIndex[iCurrentLine] < 0)
                                iArrayIndex[iCurrentLine] = 0;
                            *(char*)nxtDisplayLines[iCurrentLine].ptr = *(char*)(nxtDisplayLines[iCurrentLine].ptr + iArrayIndex[iCurrentLine]);
                            break;
                    }
                    displayClearTextLine(iCurrentLine);
				            switch(nxtDisplayLines[iCurrentLine].config){
				                case CONFIG_UNSET:
				                    break;
				                case CONFIG_CONSTANT:
				                    displayString(iCurrentLine-iOffset, "%s", nxtDisplayLines[iCurrentLine].desc);
				                    break;
				                case CONFIG_BYTE:
				                    displayString(iCurrentLine-iOffset, "%s%d%s", nxtDisplayLines[iCurrentLine].desc, *(sbyte*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
				                    break;
				                case CONFIG_SHORT:
				                    displayString(iCurrentLine-iOffset, "%s%d%s", nxtDisplayLines[iCurrentLine].desc, *(short*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
				                    break;
				                case CONFIG_LONG:
				                    displayString(iCurrentLine-iOffset, "%s%d%s", nxtDisplayLines[iCurrentLine].desc, *(long*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
				                    break;
				                case CONFIG_STRING:
				                    displayString(iCurrentLine-iOffset, "%s%s%s", nxtDisplayLines[iCurrentLine].desc, *(string*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
				                    break;
				                case CONFIG_CHAR:
				                    displayString(iCurrentLine-iOffset, "%s%c%s", nxtDisplayLines[iCurrentLine].desc, *(char*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
				                    break;
				            }
				            for(int index = 7; index > 0; index--)
			        				invertLine(0, ((7-iCurrentLine)*8)+index, 99, ((7-iCurrentLine)*8)+index);
                    wait1Msec(accelerate(300, 50, 5000, time1[T1]));
                }
            }
        }   //if(nNxtButtonPressed == 2)
        if(nNxtButtonPressed == 3){
        		clearTimer(T1);
            if(!onPress)
                while(nNxtButtonPressed == 3 && time1[T1] <= 500){}
            displayClearTextLine(iCurrentLine);
            switch(nxtDisplayLines[iCurrentLine].config){
                case CONFIG_UNSET:
                    break;
                case CONFIG_CONSTANT:
                    displayString(iCurrentLine-iOffset, "%s", nxtDisplayLines[iCurrentLine].desc);
                    break;
                case CONFIG_BYTE:
                    displayString(iCurrentLine-iOffset, "%s%d%s", nxtDisplayLines[iCurrentLine].desc, *(sbyte*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
                    break;
                case CONFIG_SHORT:
                    displayString(iCurrentLine-iOffset, "%s%d%s", nxtDisplayLines[iCurrentLine].desc, *(short*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
                    break;
                case CONFIG_LONG:
                    displayString(iCurrentLine-iOffset, "%s%d%s", nxtDisplayLines[iCurrentLine].desc, *(long*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
                    break;
                case CONFIG_BOOL:
                    sTemp = (*(bool*)nxtDisplayLines[iCurrentLine].ptr)?nxtDisplayLines[iCurrentLine].valTrue:nxtDisplayLines[iCurrentLine].valFalse;
                    displayString(iCurrentLine-iOffset, "%s%s%s", nxtDisplayLines[iCurrentLine].desc, sTemp, nxtDisplayLines[iCurrentLine].units);
                    break;
                case CONFIG_STRING:
                    displayString(iCurrentLine-iOffset, "%s%s%s", nxtDisplayLines[iCurrentLine].desc, *(string*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
                    break;
                case CONFIG_CHAR:
                    displayString(iCurrentLine-iOffset, "%s%c%s", nxtDisplayLines[iCurrentLine].desc, *(char*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
                    break;
            }
            do{
                if(++iCurrentLine > iNumLines)
                    iCurrentLine = 0;
            }while(nxtDisplayLines[iCurrentLine].config <= 1);    //Constant or unset
   			    if(false /*iCurrentLine >= 5 && iNumLines > 7*/){
                while(iCurrentLine - iOffset >= 5 && iNumLines - iCurrentLine > 2){
                    iOffset--;
                }
                for(int index = 0; index <= 7; index++){
                    switch(nxtDisplayLines[index].config){
                        case CONFIG_UNSET:
                            break;
                        case CONFIG_CONSTANT:
                            displayString(index, "%s", nxtDisplayLines[index].desc);
                            break;
                        case CONFIG_BYTE:
                            displayString(index, "%s%d%s", nxtDisplayLines[index].desc, *(sbyte*)nxtDisplayLines[index].ptr, nxtDisplayLines[index].units);
                            break;
                        case CONFIG_SHORT:
                            displayString(index, "%s%d%s", nxtDisplayLines[index].desc, *(short*)nxtDisplayLines[index].ptr, nxtDisplayLines[index].units);
                            break;
                        case CONFIG_LONG:
                            displayString(index, "%s%d%s", nxtDisplayLines[index].desc, *(long*)nxtDisplayLines[index].ptr, nxtDisplayLines[index].units);
                            break;
                        case CONFIG_BOOL:
                            sTemp = (*(bool*)nxtDisplayLines[index].ptr)?nxtDisplayLines[index].valTrue:nxtDisplayLines[index].valFalse;
                            displayString(index, "%s%s%s", nxtDisplayLines[index].desc, sTemp, nxtDisplayLines[index].units);
                            break;
                        case CONFIG_STRING:
                            displayString(index, "%s%s%s", nxtDisplayLines[index].desc, *(string*)nxtDisplayLines[index].ptr, nxtDisplayLines[index].units);
                            break;
                        case CONFIG_CHAR:
                            displayString(index, "%s%c%s", nxtDisplayLines[index].desc, *(char*)nxtDisplayLines[index].ptr, nxtDisplayLines[index].units);
                            break;
                    }
                }
            }else{
            	displayClearTextLine(iCurrentLine);
	            switch(nxtDisplayLines[iCurrentLine].config){
	                case CONFIG_UNSET:
	                    break;
	                case CONFIG_CONSTANT:
	                    displayString(iCurrentLine-iOffset, "%s", nxtDisplayLines[iCurrentLine].desc);
	                    break;
	                case CONFIG_BYTE:
	                    displayString(iCurrentLine-iOffset, "%s%d%s", nxtDisplayLines[iCurrentLine].desc, *(sbyte*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
	                    break;
	                case CONFIG_SHORT:
	                    displayString(iCurrentLine-iOffset, "%s%d%s", nxtDisplayLines[iCurrentLine].desc, *(short*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
	                    break;
	                case CONFIG_LONG:
	                    displayString(iCurrentLine-iOffset, "%s%d%s", nxtDisplayLines[iCurrentLine].desc, *(long*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
	                    break;
	                case CONFIG_BOOL:
	                    sTemp = (*(bool*)nxtDisplayLines[iCurrentLine].ptr)?nxtDisplayLines[iCurrentLine].valTrue:nxtDisplayLines[iCurrentLine].valFalse;
	                    displayString(iCurrentLine-iOffset, "%s%s%s", nxtDisplayLines[iCurrentLine].desc, sTemp, nxtDisplayLines[iCurrentLine].units);
	                    break;
	                case CONFIG_STRING:
	                    displayString(iCurrentLine-iOffset, "%s%s%s", nxtDisplayLines[iCurrentLine].desc, *(string*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
	                    break;
	                case CONFIG_CHAR:
	                    displayString(iCurrentLine-iOffset, "%s%c%s", nxtDisplayLines[iCurrentLine].desc, *(char*)nxtDisplayLines[iCurrentLine].ptr, nxtDisplayLines[iCurrentLine].units);
	                    break;
	            }
	            for(int index = 7; index > 0; index--)
        				invertLine(0, ((7-iCurrentLine)*8)+index, 99, ((7-iCurrentLine)*8)+index);
	          }
            if(onPress)
                while(nNxtButtonPressed == 3 && time1[T1] <= 500){}

            if(time1[T1] > 1000){ //Hold down the enter button for more than a second and release to exit
					    if(savePrefs){
				    		Delete(kConfigFileName, nIoResult);
				        OpenWrite(hFileHandle, nIoResult, kConfigFileName, nFileSize);
				        if(nIoResult == ioRsltSuccess){
					        for(int index = 0; index <= iNumLines; index++){
					        	switch(nxtDisplayLines[index].config) {
					     	     case CONFIG_UNSET:
					   		       break;
						          case CONFIG_CONSTANT:
							          break;
						         case CONFIG_BYTE:
							          WriteByte(hFileHandle, nIoResult, *(sbyte*)nxtDisplayLines[index].ptr);
							          break;
						          case CONFIG_SHORT:
							          WriteShort(hFileHandle, nIoResult, *(short*)nxtDisplayLines[index].ptr);
							          break;
				  	          case CONFIG_LONG:
							          WriteLong(hFileHandle, nIoResult, *(long*)nxtDisplayLines[index].ptr);
							          break;
			     				    case CONFIG_BOOL:
			      				   	WriteByte(hFileHandle, nIoResult, (*(bool*)nxtDisplayLines[index].ptr)?1:0);
			  				       	break;
						          default:
					 		          break;
					      	  	}
					        	}
				      		}
				        	Close(hFileHandle, nIoResult);
					      }
                eraseDisplay();
                sleep(1);
                eraseDisplay(); //In case failed first time
                nNxtExitClicks = 1;
#ifdef getJoystickSettings
                startTask(displayDiagnostics);
#endif
                return;
            }
        }	//if(nNxtButtonPressed == 3)
    }
}

#endif //__AUTONOMOUS_FUNCS_H__
