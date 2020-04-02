#ifndef __CASCADE__
#define __CASCADE__
void CASCADE_Init();
int CASCADE_Get_CASCADE_Mode();
float CASCADE_Get_LEADSCREW_Pos();
float CASCADE_Get_ENCODER_RPM();
float CASCADE_Get_MOTOR_Current();
float CASCADE_Get_DEMANDED_Current_Total();
float CASCADE_Get_DEMANDED_Current();
float CASCADE_Get_DEMANDED_Velocity_Total();
float CASCADE_Get_DEMANDED_Velocity();
float CASCADE_Get_DEMANDED_Position();
#endif