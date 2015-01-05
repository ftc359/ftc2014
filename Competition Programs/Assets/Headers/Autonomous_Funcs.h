#ifndef __AUTONOMOUS_FUNCS_H__
#define __AUTONOMOUS_FUNCS_H__

#pragma systemFile

#ifndef ENC_THRESHOLD       //To prevent encoder errors.
#define ENC_THRESHOLD   2000
#endif

enum tDirection{
    fwd = 0,
    left = 1,
    right = 2,
    bwd = 3
};

struct nxtDisplayLineData{
    char config;
    string desc;
    string units;
    bool *bp;
    string valTrue;
    string valFalse;
    long *lp;
    long lowerLimitL;
    long incrementL;
    long upperLimitL;
    int *ip;
    int lowerLimitI;
    int incrementI;
    int upperLimitI;
    short *sp;
    short lowerLimitS;
    short incrementS;
    short upperLimitS;
    float *fp;
    float lowerLimitF;
    float incrementF;
    float upperLimitF;
};

struct nxtDisplayLineData nxtDisplayLines[8];

//Function prototypes
void move(int power, long time, tDirection dir);
void moveEnc(int power, long distance, tDirection dir);
int accelerate(int startSpeed, int stopSpeed, long maxTime, long currentTime);

void configLine(int line, string desc, bool &input, string units, string valTrue, string valFalse);
void configLine(int line, string desc, long &input, string units, long lowerLimit, long increment, long upperLimit);
void configLine(int line, string desc, int &input, string units, int lowerLimit, int increment, int upperLimit);
void configLine(int line, string desc, short &input, string units, short lowerLimit, short increment, short upperLimit);
void configLine(int line, string desc, char &input, string units, char lowerLimit, char increment, char upperLimit);
void configLine(int line, string desc, float &input, string units, float lowerLimit, float increment, float upperLimit);
void startDisplay();

void move(int power, long time, tDirection dir){
#if defined(__HOLO_H__) && defined(DRIVEFR) && defined(DRIVEFL) && defined(DRIVEBR) && defined(DRIVEBL)
    motor[DRIVEFL] = power*(dir & 1)?-1:1;
    motor[DRIVEFR] = power*(dir & 2)?-1:1;
    motor[DRIVEBL] = power*(dir & 1)?-1:1;
    motor[DRIVEBR] = power*(dir & 2)?-1:1;
    wait1Msec(time);
    motor[DRIVEFL] = 0;
    motor[DRIVEFR] = 0;
    motor[DRIVEBL] = 0;
    motor[DRIVEBR] = 0;
#elif defined(DRIVER) && defined(DRIVEL)
    motor[DRIVEL] = power*(dir & 1)?-1:1;
    motor[DRIVER] = power*(dir & 2)?-1:1;
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
    wait1Msec(250); //wait for the encoders to calibrate
    motor[DRIVEFL] = power*(dir & 1)?-1:1;
    motor[DRIVEFR] = power*(dir & 2)?-1:1;
    motor[DRIVEBL] = power*(dir & 1)?-1:1;
    motor[DRIVEBR] = power*(dir & 2)?-1:1;
    while(nPrev[0] < distance && nPrev[1] < distance && nPrev[2] < distance && nPrev[3] < distance){
        nPrev[0] = (abs(nMotorEncoder[DRIVEFL]) >= nPrev[0] + ENC_THRESHOLD)?nPrev[0]:abs(nMotorEncoder[DRIVEFL]);
        nPrev[1] = (abs(nMotorEncoder[DRIVEFR]) >= nPrev[1] + ENC_THRESHOLD)?nPrev[1]:abs(nMotorEncoder[DRIVEFR]);
        nPrev[2] = (abs(nMotorEncoder[DRIVEBL]) >= nPrev[2] + ENC_THRESHOLD)?nPrev[2]:abs(nMotorEncoder[DRIVEBL]);
        nPrev[3] = (abs(nMotorEncoder[DRIVEBR]) >= nPrev[3] + ENC_THRESHOLD)?nPrev[3]:abs(nMotorEncoder[DRIVEBR]);
    }
    motor[DRIVEFL] = 0;
    motor[DRIVEFR] = 0;
    motor[DRIVEBL] = 0;
    motor[DRIVEBR] = 0;
#elif defined(DRIVER) && defined(DRIVEL)
    nMotorEncoder[DRIVEL] = 0;
    nMotorEncoder[DRIVER] = 0;
    wait1Msec(250); //wait for the encoders to calibrate
    motor[DRIVEL] = power*(dir & 1)?-1:1;
    motor[DRIVER] = power*(dir & 2)?-1:1;
    while(nPrev[0] < distance && nPrev[1] < distance){
        nPrev[0] = (abs(nMotorEncoder[DRIVEL]) >= nPrev[0] + ENC_THRESHOLD)?nPrev[0]:abs(nMotorEncoder[DRIVEL]);
        nPrev[1] = (abs(nMotorEncoder[DRIVER]) >= nPrev[1] + ENC_THRESHOLD)?nPrev[1]:abs(nMotorEncoder[DRIVER]);
    }
    motor[DRIVEL] = 0;
    motor[DRIVER] = 0;
#endif
}
int accelerate(int startSpeed, int stopSpeed, long maxTime, long currentTime){
	return ((startSpeed-stopSpeed)/pow(maxTime,2))*pow(currentTime-maxTime,2)+stopSpeed;
}

void configLine(int line, string desc, bool &input, string units, string valTrue, string valFalse){
    nxtDisplayLines[line].config = 1;
    nxtDisplayLines[line].desc = desc;
    nxtDisplayLines[line].bp = &input;
    nxtDisplayLines[line].units = units;
    nxtDisplayLines[line].valTrue = valTrue;
    nxtDisplayLines[line].valFalse = valFalse;
}
void configLine(int line, string desc, long &input, string units, long lowerLimit, long increment, long upperLimit){
    nxtDisplayLines[line].config = 2;
    nxtDisplayLines[line].desc = desc;
    nxtDisplayLines[line].lp = &input;
    nxtDisplayLines[line].units = units;
    nxtDisplayLines[line].lowerLimitL = lowerLimit;
    nxtDisplayLines[line].incrementL = increment;
    nxtDisplayLines[line].upperLimitL = upperLimit;
}
void configLine(int line, string desc, int &input, string units, int lowerLimit, int increment, int upperLimit){
    nxtDisplayLines[line].config = 3;
    nxtDisplayLines[line].desc = desc;
    nxtDisplayLines[line].ip = &input;
    nxtDisplayLines[line].units = units;
    nxtDisplayLines[line].lowerLimitI = lowerLimit;
    nxtDisplayLines[line].incrementI = increment;
    nxtDisplayLines[line].upperLimitI = upperLimit;
}
void configLine(int line, string desc, short &input, string units, short lowerLimit, short increment, short upperLimit){
    nxtDisplayLines[line].config = 4;
    nxtDisplayLines[line].desc = desc;
    nxtDisplayLines[line].sp = &input;
    nxtDisplayLines[line].units = units;
    nxtDisplayLines[line].lowerLimitS = lowerLimit;
    nxtDisplayLines[line].incrementS = increment;
    nxtDisplayLines[line].upperLimitS = upperLimit;
}
void configLine(int line, string desc, float &input, string units, float lowerLimit, float increment, float upperLimit){
    nxtDisplayLines[line].config = 5;
    nxtDisplayLines[line].desc = desc;
    nxtDisplayLines[line].fp = &input;
    nxtDisplayLines[line].units = units;
    nxtDisplayLines[line].lowerLimitF = lowerLimit;
    nxtDisplayLines[line].incrementF = increment;
    nxtDisplayLines[line].upperLimitF = upperLimit;
}
void startDisplay(){
    //Initialize local variables
    int nSelectedLine = (nxtDisplayLines[0].config)?0:(nxtDisplayLines[1].config)?1:
                        (nxtDisplayLines[2].config)?2:(nxtDisplayLines[3].config)?3:
                        (nxtDisplayLines[4].config)?4:(nxtDisplayLines[5].config)?5:
                        (nxtDisplayLines[6].config)?6:7;
    byte ignoreNXTBut;    //Single byte to use with bitmasking
    byte recent;
    bool bValChanged;
    //Initialize display
    nNxtExitClicks += 1; //not = 2 in case it is user set to something higher already
#ifdef getJoystickSettings  //Check if "JoystickDriver.c" is included
    bDisplayDiagnostics = false; //Turn off NXT stat info in order to access the LCD screen
    wait1Msec(200);
#endif
    eraseDisplay();
    for(int line; line < 8; line++){
        switch(nxtDisplayLines[line].config){
            case 1:
                displayString(line, "%s%s%s", nxtDisplayLines[line].desc, *nxtDisplayLines[line].bp?nxtDisplayLines[line].valTrue:nxtDisplayLines[line].valFalse, nxtDisplayLines[line].units);
                break;
            case 2:
                displayString(line, "%s%d%s", nxtDisplayLines[line].desc, *nxtDisplayLines[line].lp, nxtDisplayLines[line].units);
                break;
            case 3:
                displayString(line, "%s%d%s", nxtDisplayLines[line].desc, *nxtDisplayLines[line].ip, nxtDisplayLines[line].units);
                break;
            case 4:
                displayString(line, "%s%d%s", nxtDisplayLines[line].desc, *nxtDisplayLines[line].sp, nxtDisplayLines[line].units);
                break;
            case 5:
                displayString(line, "%s%.3f%s", nxtDisplayLines[line].desc, *nxtDisplayLines[line].fp, nxtDisplayLines[line].units);
                break;
            default:    //Line has not been set!
                break;
        }
    }
    //Begin reading for button interactions
    while(true){
        if(nNxtButtonPressed == 0 && !(ignoreNXTBut&1))
            ignoreNXTBut |= 1;
        else if(nNxtButtonPressed != 0 && (ignoreNXTBut&1)){
            ignoreNXTBut ^= 1;
            break;  //Exit
        }
        if(nNxtButtonPressed == 3 && !(ignoreNXTBut&4))
            ignoreNXTBut |= 4;
        else if(nNxtButtonPressed != 3 && ignoreNXTBut&4){
            switch(nxtDisplayLines[nSelectedLine].config){
                case 1:
                    displayString(nSelectedLine, "%s%s%s", nxtDisplayLines[nSelectedLine].desc, *nxtDisplayLines[nSelectedLine].bp?nxtDisplayLines[nSelectedLine].valTrue:nxtDisplayLines[nSelectedLine].valFalse, nxtDisplayLines[nSelectedLine].units);
                    break;
                case 2:
                    displayString(nSelectedLine, "%s%d%s", nxtDisplayLines[nSelectedLine].desc, *nxtDisplayLines[nSelectedLine].lp, nxtDisplayLines[nSelectedLine].units);
                    break;
                case 3:
                    displayString(nSelectedLine, "%s%d%s", nxtDisplayLines[nSelectedLine].desc, *nxtDisplayLines[nSelectedLine].ip, nxtDisplayLines[nSelectedLine].units);
                    break;
                case 4:
                    displayString(nSelectedLine, "%s%d%s", nxtDisplayLines[nSelectedLine].desc, *nxtDisplayLines[nSelectedLine].sp, nxtDisplayLines[nSelectedLine].units);
                    break;
                case 5:
                    displayString(nSelectedLine, "%s%.3f%s", nxtDisplayLines[nSelectedLine].desc, *nxtDisplayLines[nSelectedLine].fp, nxtDisplayLines[nSelectedLine].units);
                    break;
                default:    //Line has not been set!
                    break;
            }
            do{
                if(++nSelectedLine > 7)
                    nSelectedLine = (nxtDisplayLines[0].config)?0:(nxtDisplayLines[1].config)?1:
                                    (nxtDisplayLines[2].config)?2:(nxtDisplayLines[3].config)?3:
                                    (nxtDisplayLines[4].config)?4:(nxtDisplayLines[5].config)?5:
                                    (nxtDisplayLines[6].config)?6:7;
            }while(!nxtDisplayLines[nSelectedLine].config);
            bValChanged = true;
            ignoreNXTBut ^= 4;
        }
        if(nNxtButtonPressed == 1 && !(ignoreNXTBut&2)){
            if(nxtDisplayLines[nSelectedLine].config == 1)
                ignoreNXTBut |= 2;
            else if(time1[T4] >= accelerate(250,100,2000,time1[T3])){
                if(recent&1){
                    clearTimer(T3);
                    recent |= 1;
                }
                clearTimer(T4);
                switch(nxtDisplayLines[nSelectedLine].config){
                    case 2:
                        *nxtDisplayLines[nSelectedLine].lp += nxtDisplayLines[nSelectedLine].incrementL;
                        if(*nxtDisplayLines[nSelectedLine].lp > nxtDisplayLines[nSelectedLine].upperLimitL)
                            *nxtDisplayLines[nSelectedLine].lp = nxtDisplayLines[nSelectedLine].upperLimitL;
                        break;
                    case 3:
                        *nxtDisplayLines[nSelectedLine].ip += nxtDisplayLines[nSelectedLine].incrementI;
                        if(*nxtDisplayLines[nSelectedLine].ip > nxtDisplayLines[nSelectedLine].upperLimitI)
                            *nxtDisplayLines[nSelectedLine].ip = nxtDisplayLines[nSelectedLine].upperLimitI;
                        break;
                    case 4:
                        *nxtDisplayLines[nSelectedLine].sp += nxtDisplayLines[nSelectedLine].incrementS;
                        if(*nxtDisplayLines[nSelectedLine].sp > nxtDisplayLines[nSelectedLine].upperLimitS)
                            *nxtDisplayLines[nSelectedLine].sp = nxtDisplayLines[nSelectedLine].upperLimitS;
                        break;
                    case 5:
                        *nxtDisplayLines[nSelectedLine].fp += nxtDisplayLines[nSelectedLine].incrementF;
                        if(*nxtDisplayLines[nSelectedLine].fp > nxtDisplayLines[nSelectedLine].upperLimitF)
                            *nxtDisplayLines[nSelectedLine].fp = nxtDisplayLines[nSelectedLine].upperLimitF;
                        break;
                    default:
                        break;
                }
            }
        }else if(nNxtButtonPressed != 1){
            if(ignoreNXTBut&2){
                *nxtDisplayLines[nSelectedLine].bp = !*nxtDisplayLines[nSelectedLine].bp;
                bValChanged = true;
                ignoreNXTBut ^= 2;
            }
            if(recent&1){
                bValChanged = true;
                recent ^= 1;
            }
        }
        if(nNxtButtonPressed == 2 && !(ignoreNXTBut&3)){
            if(nxtDisplayLines[nSelectedLine].config == 1)
                ignoreNXTBut |= 3;
            else if(time1[T4] >= accelerate(250,100,2000,time1[T3])){
                if(recent&2){
                    clearTimer(T3);
                    recent |= 2;
                }
                clearTimer(T4);
                switch(nxtDisplayLines[nSelectedLine].config){
                    case 2:
                        *nxtDisplayLines[nSelectedLine].lp -= nxtDisplayLines[nSelectedLine].incrementL;
                        if(*nxtDisplayLines[nSelectedLine].lp < nxtDisplayLines[nSelectedLine].lowerLimitL)
                            *nxtDisplayLines[nSelectedLine].lp = nxtDisplayLines[nSelectedLine].lowerLimitL;
                        break;
                    case 3:
                        *nxtDisplayLines[nSelectedLine].ip -= nxtDisplayLines[nSelectedLine].incrementI;
                        if(*nxtDisplayLines[nSelectedLine].ip < nxtDisplayLines[nSelectedLine].lowerLimitI)
                            *nxtDisplayLines[nSelectedLine].ip = nxtDisplayLines[nSelectedLine].lowerLimitI;
                        break;
                    case 4:
                        *nxtDisplayLines[nSelectedLine].sp -= nxtDisplayLines[nSelectedLine].incrementS;
                        if(*nxtDisplayLines[nSelectedLine].sp < nxtDisplayLines[nSelectedLine].lowerLimitS)
                            *nxtDisplayLines[nSelectedLine].sp = nxtDisplayLines[nSelectedLine].lowerLimitS;
                        break;
                    case 5:
                        *nxtDisplayLines[nSelectedLine].fp -= nxtDisplayLines[nSelectedLine].incrementF;
                        if(*nxtDisplayLines[nSelectedLine].fp < nxtDisplayLines[nSelectedLine].lowerLimitF)
                            *nxtDisplayLines[nSelectedLine].fp = nxtDisplayLines[nSelectedLine].lowerLimitF;
                        break;
                    default:
                        break;
                }
            }
        }else if(nNxtButtonPressed != 2){
            if(ignoreNXTBut&3){
                *nxtDisplayLines[nSelectedLine].bp = !*nxtDisplayLines[nSelectedLine].bp;
                bValChanged = true;
                ignoreNXTBut ^= 3;
            }
            if(recent&2){
                bValChanged = true;
                recent ^= 2;
            }
        }
        if(time1[T1] > 1000 || bValChanged){
            switch(nxtDisplayLines[nSelectedLine].config){
                case 1:
                    displayString(nSelectedLine, "%s%s%s", nxtDisplayLines[nSelectedLine].desc, *nxtDisplayLines[nSelectedLine].bp?nxtDisplayLines[nSelectedLine].valTrue:nxtDisplayLines[nSelectedLine].valFalse, nxtDisplayLines[nSelectedLine].units);
                    break;
                case 2:
                    displayString(nSelectedLine, "%s%d%s", nxtDisplayLines[nSelectedLine].desc, *nxtDisplayLines[nSelectedLine].lp, nxtDisplayLines[nSelectedLine].units);
                    break;
                case 3:
                    displayString(nSelectedLine, "%s%d%s", nxtDisplayLines[nSelectedLine].desc, *nxtDisplayLines[nSelectedLine].ip, nxtDisplayLines[nSelectedLine].units);
                    break;
                case 4:
                    displayString(nSelectedLine, "%s%d%s", nxtDisplayLines[nSelectedLine].desc, *nxtDisplayLines[nSelectedLine].sp, nxtDisplayLines[nSelectedLine].units);
                    break;
                case 5:
                    displayString(nSelectedLine, "%s%.3f%s", nxtDisplayLines[nSelectedLine].desc, *nxtDisplayLines[nSelectedLine].fp, nxtDisplayLines[nSelectedLine].units);
                    break;
                default:    //Line has not been set!
                    break;
            }
            if(bValChanged)
                bValChanged = false;
            clearTimer(T1);
        }else if(time1[T1] > 500)
            displayClearTextLine(nSelectedLine);
#ifdef
        getJoystickSettings(joystick);
        if (!joystick.StopPgm)
            break;
#endif
    }
#ifdef getJoystickSettings
    bDisplayDiagnostics = true; //Resume NXT status display
#endif
}

#ifdef getJoystickSettings
void waitForStart(bool saveCPU, bool allowDisplayChanges)
{
    while (true)
    {
        getJoystickSettings(joystick);
        if (!joystick.StopPgm)
            break;
        if(allowDisplayChanges && nNxtButtonPressed == 3){
            while(nNxtButtonPressed == 3){/*Do Nothing*/}
            startDisplay();
        }
    }
    if(saveCPU)
        stopTask(readMsgFromPC);
    return;
}
#endif

#endif //__AUTONOMOUS_FUNCS_H__
