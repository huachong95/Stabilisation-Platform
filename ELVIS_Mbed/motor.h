#ifndef __MOTOR__
#define __MOTOR__
void MOTOR_Init();
void MOTOR_SetSpeed(int data1,int data2);
int MOTOR_GetLSpeed();
int MOTOR_GetRSpeed();
int MOTOR_GetLPosition();
int MOTOR_GetRPosition();
int MOTOR_GetBatteryVoltage();
void MOTOR_Test();

#endif