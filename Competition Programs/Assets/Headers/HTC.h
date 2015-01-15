#ifndef __HTC_H__
#define __HTC_H__

#pragma systemFile

#ifndef __COMMON_H__
#include "Xander's Drivers\common.h"
#endif

#define HTC_I2C_ADDR_1      0x02 //Address of the first controller
#define HTC_I2C_ADDR(CHAN)  HTC_I2C_ADDR_1*CHAN

#define HTC_TYPE            0x10

#define HTMC_OFFSET         0x44
#define HTMC_POWER          0x00
#define HTMC_MODE           0x01
#define HTMC_M_ENTRY_SIZE   0x01

#define HTSC_OFFSET					0x40
#define HTSC_POS_OFFSET			0x02
#define HTSC_POS(CHAN)      0x01*CHAN
#define HTSC_STEP_TIME      0x01
#define HTSC_PWM_ENABLE			0x08

tByteArray HTC_I2CRequest;
tByteArray HTC_I2CReply;

enum tMC{
    motor_S1_C1_1 = 0,
    motor_S1_C1_2 = 1,
    motor_S1_C2_1 = 2,
    motor_S1_C2_2 = 3,
    motor_S1_C3_1 = 4,
    motor_S1_C3_2 = 5,
    motor_S1_C4_1 = 6,
    motor_S1_C4_2 = 7,
    motor_S2_C1_1 = 8,
    motor_S2_C1_2 = 9,
    motor_S2_C2_1 = 10,
    motor_S2_C2_2 = 11,
    motor_S2_C3_1 = 12,
    motor_S2_C3_2 = 13,
    motor_S2_C4_1 = 14,
    motor_S2_C4_2 = 15,
    motor_S3_C1_1 = 16,
    motor_S3_C1_2 = 17,
    motor_S3_C2_1 = 18,
    motor_S3_C2_2 = 19,
    motor_S3_C3_1 = 20,
    motor_S3_C3_2 = 21,
    motor_S3_C4_1 = 22,
    motor_S3_C4_2 = 23,
    motor_S4_C1_1 = 24,
    motor_S4_C1_2 = 25,
    motor_S4_C2_1 = 26,
    motor_S4_C2_2 = 27,
    motor_S4_C3_1 = 28,
    motor_S4_C3_2 = 29,
    motor_S4_C4_1 = 30,
    motor_S4_C4_2 = 31
};

enum tSC{
    servo_S1_C1_1 = 0,
    servo_S1_C1_2 = 1,
    servo_S1_C1_3 = 2,
    servo_S1_C1_4 = 3,
    servo_S1_C1_5 = 4,
    servo_S1_C1_6 = 5,
    servo_S1_C2_1 = 6,
    servo_S1_C2_2 = 7,
    servo_S1_C2_3 = 8,
    servo_S1_C2_4 = 9,
    servo_S1_C2_5 = 10,
    servo_S1_C2_6 = 11,
    servo_S1_C3_1 = 12,
    servo_S1_C3_2 = 13,
    servo_S1_C3_3 = 14,
    servo_S1_C3_4 = 15,
    servo_S1_C3_5 = 16,
    servo_S1_C3_6 = 17,
    servo_S1_C4_1 = 18,
    servo_S1_C4_2 = 19,
    servo_S1_C4_3 = 20,
    servo_S1_C4_4 = 21,
    servo_S1_C4_5 = 22,
    servo_S1_C4_6 = 23,
    servo_S2_C1_1 = 24,
    servo_S2_C1_2 = 25,
    servo_S2_C1_3 = 26,
    servo_S2_C1_4 = 27,
    servo_S2_C1_5 = 28,
    servo_S2_C1_6 = 29,
    servo_S2_C2_1 = 30,
    servo_S2_C2_2 = 31,
    servo_S2_C2_3 = 32,
    servo_S2_C2_4 = 33,
    servo_S2_C2_5 = 34,
    servo_S2_C2_6 = 35,
    servo_S2_C3_1 = 36,
    servo_S2_C3_2 = 37,
    servo_S2_C3_3 = 38,
    servo_S2_C3_4 = 39,
    servo_S2_C3_5 = 40,
    servo_S2_C3_6 = 41,
    servo_S2_C4_1 = 42,
    servo_S2_C4_2 = 43,
    servo_S2_C4_3 = 44,
    servo_S2_C4_4 = 45,
    servo_S2_C4_5 = 46,
    servo_S2_C4_6 = 47,
    servo_S3_C1_1 = 48,
    servo_S3_C1_2 = 49,
    servo_S3_C1_3 = 50,
    servo_S3_C1_4 = 51,
    servo_S3_C1_5 = 52,
    servo_S3_C1_6 = 53,
    servo_S3_C2_1 = 54,
    servo_S3_C2_2 = 55,
    servo_S3_C2_3 = 56,
    servo_S3_C2_4 = 57,
    servo_S3_C2_5 = 58,
    servo_S3_C2_6 = 59,
    servo_S3_C3_1 = 60,
    servo_S3_C3_2 = 61,
    servo_S3_C3_3 = 62,
    servo_S3_C3_4 = 63,
    servo_S3_C3_5 = 64,
    servo_S3_C3_6 = 65,
    servo_S3_C4_1 = 66,
    servo_S3_C4_2 = 67,
    servo_S3_C4_3 = 68,
    servo_S3_C4_4 = 69,
    servo_S3_C4_5 = 70,
    servo_S3_C4_6 = 71,
    servo_S4_C1_1 = 72,
    servo_S4_C1_2 = 73,
    servo_S4_C1_3 = 74,
    servo_S4_C1_4 = 75,
    servo_S4_C1_5 = 76,
    servo_S4_C1_6 = 77,
    servo_S4_C2_1 = 78,
    servo_S4_C2_2 = 79,
    servo_S4_C2_3 = 80,
    servo_S4_C2_4 = 81,
    servo_S4_C2_5 = 82,
    servo_S4_C2_6 = 83,
    servo_S4_C3_1 = 84,
    servo_S4_C3_2 = 85,
    servo_S4_C3_3 = 86,
    servo_S4_C3_4 = 87,
    servo_S4_C3_5 = 88,
    servo_S4_C3_6 = 89,
    servo_S4_C4_1 = 90,
    servo_S4_C4_2 = 91,
    servo_S4_C4_3 = 92,
    servo_S4_C4_4 = 93,
    servo_S4_C4_5 = 94,
    servo_S4_C4_6 = 95
};

enum tSCPWM{
	reset_timeout = 0x00,
	disable_timeout = 0xAA,
	force_timeout = 0xFF
};

int HTCVerifyType(tSensors link, ubyte channel){
    memset(HTC_I2CRequest, 0, sizeof(tByteArray));
    HTC_I2CRequest[0] = 2;
    HTC_I2CRequest[1] = HTC_I2C_ADDR(channel);
    HTC_I2CRequest[2] = HTC_TYPE;

    if(!writeI2C(link, HTC_I2CRequest, HTC_I2CReply, 8))
        return -1;

    if((char)HTC_I2CReply[0] == 'M' && (char)HTC_I2CReply[1] == 'o' &&
       (char)HTC_I2CReply[2] == 't' && (char)HTC_I2CReply[3] == 'o' &&
       (char)HTC_I2CReply[4] == 'r' && (char)HTC_I2CReply[5] == 'C' &&
       (char)HTC_I2CReply[6] == 'o' && (char)HTC_I2CReply[7] == 'n')

        return 1;

    if((char)HTC_I2CReply[0] == 'S' && (char)HTC_I2CReply[1] == 'e' &&
       (char)HTC_I2CReply[2] == 'r' && (char)HTC_I2CReply[3] == 'v' &&
       (char)HTC_I2CReply[4] == 'o' && (char)HTC_I2CReply[5] == 'C' &&
       (char)HTC_I2CReply[6] == 'o' && (char)HTC_I2CReply[7] == 'n')

        return 2;

    return 0;
}

bool HTMCPower(tSensors link, tMC mot, byte power){
    memset(HTC_I2CRequest, 0, sizeof(tByteArray));
    int controller_channel = (int)(floor(mot/2)%4)+1;
    int motor_channel = (mot%2) + 1;

    HTC_I2CRequest[0] = 3;
    HTC_I2CRequest[1] = HTC_I2C_ADDR(controller_channel);
    HTC_I2CRequest[2] = HTMC_OFFSET + (HTMC_M_ENTRY_SIZE*motor_channel) + HTMC_POWER;
    HTC_I2CRequest[3] = power;

    return writeI2C(link, HTC_I2CRequest);
}

bool HTSCServo(tSensors link, tSC ser, ubyte pos, tSCPWM mode){
	  memset(HTC_I2CRequest, 0, sizeof(tByteArray));
    int controller_channel = (int)(floor(ser/6)%4)+1;
    int servo_channel = ser%6;

    HTC_I2CRequest[0] = 2;
    HTC_I2CRequest[1] = HTC_I2C_ADDR(controller_channel);
    HTC_I2CRequest[2] = HTSC_OFFSET + HTSC_PWM_ENABLE;

    writeI2C(link, HTC_I2CRequest, HTC_I2CReply, 1);

    if((int)HTC_I2CReply[0] != mode){
	    memset(HTC_I2CRequest, 0, sizeof(tByteArray));

	    HTC_I2CRequest[0] = 3;
	    HTC_I2CRequest[1] = HTC_I2C_ADDR(controller_channel);
	    HTC_I2CRequest[2] = HTSC_OFFSET + HTSC_PWM_ENABLE;
	    HTC_I2CRequest[3] = mode;

	    writeI2C(link, HTC_I2CRequest);
	  }

    memset(HTC_I2CRequest, 0, sizeof(tByteArray));

    HTC_I2CRequest[0] = 3;
    HTC_I2CRequest[1] = HTC_I2C_ADDR(controller_channel);
    HTC_I2CRequest[2] = HTSC_OFFSET + HTSC_POS_OFFSET + HTSC_POS(servo_channel);
    HTC_I2CRequest[3] = pos;

    return writeI2C(link, HTC_I2CRequest);
}
#endif //__HTC_H__
