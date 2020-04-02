/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */
#define IMU_INTERVAL 0.01 // 100Hz
#define SERIAL_PRINT_INTERVAL 0.01
#define SAMPLING_TIME 0.01 // 100Hz

#include "BNO055.h"
#include "cmath"
#include "mbed.h"
#include "platform/mbed_thread.h"

RawSerial PC(USBTX, USBRX, 115200); // tx, rx for CoolTerm output
DigitalOut led(LED1);
BNO055 imu1(D14, D15);
BNO055 imu2(D5, D7);
Ticker ANGLE_ISR;
// Ticker FD_Computation_ISR;
Ticker SERIAL_PRINT;
Timer t;

float IMU1_Pitch = 0.0;
float IMU1_Roll = 0.0;
float IMU1_X_Acc = 0.0;
float IMU1_Y_Acc = 0.0;
float IMU1_Z_Acc = 0.0;
float IMU1_X_Linear_Acc = 0.0;
float IMU1_Y_Linear_Acc = 0.0;
float IMU1_Z_Linear_Acc = 0.0;

float IMU2_Pitch = 0.0;
float IMU2_Roll = 0.0;
float IMU2_X_Acc = 0.0;
float IMU2_Y_Acc = 0.0;
float IMU2_Z_Acc = 0.0;
float IMU2_X_Linear_Acc = 0.0;
float IMU2_Y_Linear_Acc = 0.0;
float IMU2_Z_Linear_Acc = 0.0;

bool SERIAL_Print_Flag = 0;
bool IMU_Flag = 0;
float PEN_Angle = 0.0;

float x_ddot = 0.0;
float y_ddot = 0.0;

float Y_DDOT = 0.0;
float Z_DDOT = 0.0;
float Z_DDOT_Fil5 = 0.0;
float Z_DDOT_Fil4 = 0.0;
float Z_DDOT_Fil3 = 0.0;
float Z_DDOT_Fil2 = 0.0;
float Y_DDOT_Fil4 = 0.0;

float LENGTH_Acc = 0.0;
// Finite Difference Implementation Variables
float FD_Acc_y[4] = {0, 0, 0, 0};
float FD_Acc_u[4] = {0, 0, 0, 0};
float FD_OutputAcc[2] = {0, 0};
float FD_OutputVel[2] = {0, 0};
float FD_OutputPos[2] = {0, 0}; // NEED TO CHANGE THIS TO INITIAL POSITION
float FD_OutputAcc_Fil[2] = {0, 0};
float FD_OutputVel_Fil[2] = {0, 0};
float FD_OutputPos_Fil[2] = {0, 0};
float FD_OutputVel_Unfiltered[2] = {0, 0};
float FD_OutputPos_Unfiltered[2] = {0, 0};
float MA5_Data[5] = {0, 0, 0, 0, 0};
float MA4_Data1[4] = {0, 0, 0, 0};
float MA4_Data2[4] = {0, 0, 0, 0};
float MA3_Data[3] = {0, 0, 0};
float MA2_Data[2] = {0, 0};

const float A_filter_alpha = 0.975;
const float V_filter_alpha = 1;
const float P_filter_alpha = 0.99;
void IMU_Init();
void IMU1_Angle();
void IMU2_Angle();
void IMU1_Acceleration();
void IMU2_Acceleration();
void IMU1_Linear_Acceleration();
void IMU2_Linear_Acceleration();
void Acceleration_Computation();
void IMU_ISR();
void SERIAL_Print();
void SERIAL_Print_ISR();
void FD_Computation();
float Moving_Average5(float data);
float Moving_Average4_1(float data);
float Moving_Average4_2(float data);
float Moving_Average3(float data);
float Moving_Average2(float data);
float map(float in, float inMin, float inMax, float outMin, float outMax);

int main() {
  t.start();
  PC.baud(115200);
  led = 1;
  // Reset the BNO055
  imu1.reset();
  imu2.reset();
  IMU_Init();
  ANGLE_ISR.attach(&IMU_ISR, IMU_INTERVAL);
  SERIAL_PRINT.attach(&SERIAL_Print_ISR, SERIAL_PRINT_INTERVAL);
//   FD_Computation_ISR.attach(&FD_Computation, SAMPLING_TIME);

  while (true) {
    imu1.setmode(OPERATION_MODE_NDOF);
    imu2.setmode(OPERATION_MODE_NDOF);

    if (IMU_Flag) {
      IMU1_Angle();
      IMU2_Angle();
      //   IMU1_Acceleration();
      IMU1_Linear_Acceleration();
      Acceleration_Computation();
      FD_Computation();

      IMU_Flag = 0;
    }
    if (SERIAL_Print_Flag) {
      SERIAL_Print();
      SERIAL_Print_Flag = 0;
    }
  }
}

float map(float in, float inMin, float inMax, float outMin,
          float outMax) { // Function to scale the inputs to desired outputs
  // check it's within the range
  if (inMin < inMax) {
    if (in <= inMin)
      return outMin;
    if (in >= inMax)
      return outMax;
  } else { // cope with input range being backwards.
    if (in >= inMin)
      return outMin;
    if (in <= inMax)
      return outMax;
  }
  // calculate how far into the range we are
  float scale = (in - inMin) / (inMax - inMin);
  // calculate the output.
  return outMin + scale * (outMax - outMin);
}

void IMU_Init() {
  imu1.set_accel_units(MPERSPERS); // Sets Acceleration output to m/s2
  imu1.set_angle_units(DEGREES);   // Sets Angle output to degrees
  imu2.set_accel_units(MPERSPERS); // Sets Acceleration output to m/s2
  imu2.set_angle_units(DEGREES);   // Sets Angle output to degrees
}

void IMU1_Angle() {
  imu1.get_angles();
  IMU1_Pitch = imu1.euler.pitch;
  IMU1_Roll = imu1.euler.roll;
  //   PEN_Angle = sqrt((IMU1_Pitch * IMU1_Pitch) + (IMU1_Roll * IMU1_Roll));
}
void IMU2_Angle() {
  imu2.get_angles();
  IMU2_Pitch = imu2.euler.pitch;
  IMU2_Roll = imu2.euler.roll;
  PEN_Angle = sqrt((IMU2_Pitch * IMU2_Pitch) + (IMU2_Roll * IMU2_Roll));
}

void IMU1_Acceleration() {
  imu1.get_accel();
  IMU1_X_Acc = imu1.accel.x;
  IMU1_Y_Acc = imu1.accel.y;
  IMU1_Z_Acc = imu1.accel.z;
}

void IMU2_Acceleration() {
  imu2.get_accel();
  IMU2_X_Acc = imu2.accel.x;
  IMU2_Y_Acc = imu2.accel.y;
  IMU2_Z_Acc = imu2.accel.z;
}

void IMU1_Linear_Acceleration() {
  imu1.get_lia();
  IMU1_X_Linear_Acc = imu1.lia.x;
  IMU1_Y_Linear_Acc = imu1.lia.y;
  IMU1_Z_Linear_Acc = imu1.lia.z;
}

void IMU2_Linear_Acceleration() {
  imu2.get_lia();
  IMU2_X_Linear_Acc = imu2.lia.x;
  IMU2_Y_Linear_Acc = imu2.lia.y;
  IMU2_Z_Linear_Acc = imu2.lia.z;
}

void Acceleration_Computation() {
  Z_DDOT =
      IMU1_Y_Linear_Acc * sin(IMU1_Pitch) - IMU1_X_Linear_Acc * sin(IMU1_Roll) +
      IMU1_Z_Linear_Acc * cos(IMU1_Pitch) + IMU1_Z_Linear_Acc * cos(IMU1_Roll);

  y_ddot =
      IMU1_Y_Linear_Acc * cos(IMU1_Pitch) - IMU1_Z_Linear_Acc * sin(IMU1_Pitch);

  x_ddot =
      IMU1_X_Linear_Acc * cos(IMU1_Roll) + IMU1_Z_Linear_Acc * sin(IMU1_Roll);

  Y_DDOT = sqrt((x_ddot * x_ddot) + (y_ddot * y_ddot));
  //   Z_DDOT_Fil5 = Moving_Average5(Z_DDOT);
  Z_DDOT_Fil4 = Moving_Average4_1(Z_DDOT);
  //   Z_DDOT_Fil3 = Moving_Average3(Z_DDOT);
  //   Z_DDOT_Fil2 = Moving_Average2(Z_DDOT);
  Y_DDOT_Fil4 = Moving_Average4_2(Y_DDOT);
  LENGTH_Acc = -Y_DDOT_Fil4 * sin(PEN_Angle) + Z_DDOT_Fil5 * cos(PEN_Angle);
}
void FD_Computation() {
  FD_Acc_u[0] = LENGTH_Acc; // Updates new acceleration result
  FD_Acc_y[0] = 0.9038 * FD_Acc_u[0] - 0.2216 * FD_Acc_u[1] -
                0.9049 * FD_Acc_u[2] + 0.2205 * FD_Acc_u[3] +
                0.03077 * FD_Acc_y[1] + 0.8471 * FD_Acc_y[2] -
                0.1837 * FD_Acc_y[3];
  FD_OutputAcc[0] = FD_Acc_y[0]; // Stores the result of Washout Filter
  FD_Acc_u[3] = FD_Acc_u[2];     // Shifts the acceleration results
  FD_Acc_u[2] = FD_Acc_u[1];
  FD_Acc_u[1] = FD_Acc_u[0];
  FD_Acc_y[3] = FD_Acc_y[2];
  FD_Acc_y[2] = FD_Acc_y[1];
  FD_Acc_y[1] = FD_Acc_y[0];

  FD_OutputAcc_Fil[0] = A_filter_alpha * (FD_OutputAcc_Fil[1] +
                                          FD_OutputAcc[0] - FD_OutputAcc[1]);

  FD_OutputVel[0] =
      SAMPLING_TIME / 2 * (FD_OutputAcc_Fil[0] + FD_OutputAcc_Fil[1]) +
      FD_OutputVel[1];

  FD_OutputPos[0] =
      SAMPLING_TIME / 2 * (FD_OutputVel[0] + FD_OutputVel[1]) + FD_OutputPos[1];

//   FD_OutputVel_Unfiltered[0] =
//       SAMPLING_TIME / 2 * (FD_OutputAcc[0] + FD_OutputAcc[1]) +
//       FD_OutputVel_Unfiltered[1];
  //   FD_OutputVel_Fil[0] =
  //       V_filter_alpha *
  //       (FD_OutputVel_Fil[1] + FD_OutputVel[0] - FD_OutputVel[1]);
//   FD_OutputPos_Unfiltered[0] =
//       SAMPLING_TIME / 2 *
//           (FD_OutputVel_Unfiltered[0] + FD_OutputVel_Unfiltered[1]) +
//       FD_OutputPos_Unfiltered[1];
  //   FD_OutputPos[0] =
  //       SAMPLING_TIME / 2 * (FD_OutputVel_Fil[0] + FD_OutputVel_Fil[1]) +
  //       FD_OutputPos[1];
    FD_OutputPos_Fil[0] = P_filter_alpha * (FD_OutputPos_Fil[1] +
                                            FD_OutputPos[0] -
                                            FD_OutputPos[1]);
  FD_OutputAcc[1] = FD_OutputAcc[0];
  FD_OutputVel[1] = FD_OutputVel[0]; // Shifts the velocity results
  FD_OutputPos[1] = FD_OutputPos[0]; // Shifts the position results
  FD_OutputAcc_Fil[1] = FD_OutputAcc_Fil[0];
//   FD_OutputVel_Fil[1] = FD_OutputVel_Fil[0];
  FD_OutputPos_Fil[1] = FD_OutputPos_Fil[0];
//   FD_OutputVel_Unfiltered[1] = FD_OutputVel_Unfiltered[0];
//   FD_OutputPos_Unfiltered[1] = FD_OutputPos_Unfiltered[0];
}

float Moving_Average5(float data) {
  MA5_Data[0] =
      (data + MA5_Data[1] + MA5_Data[2] + MA5_Data[3] + MA5_Data[4]) / 5;
  MA5_Data[4] = MA5_Data[3];
  MA5_Data[3] = MA5_Data[2];
  MA5_Data[2] = MA5_Data[1];
  MA5_Data[1] = MA5_Data[0];
  return MA5_Data[0];
}
float Moving_Average4_1(float data) {
  MA4_Data1[0] = (data + MA4_Data1[1] + MA4_Data1[2] + MA4_Data1[3]) / 4;
  MA4_Data1[3] = MA4_Data1[2];
  MA4_Data1[2] = MA4_Data1[1];
  MA4_Data1[1] = MA4_Data1[0];
  return MA4_Data1[0];
}
float Moving_Average4_2(float data) {
  MA4_Data2[0] = (data + MA4_Data2[1] + MA4_Data2[2] + MA4_Data2[3]) / 4;
  MA4_Data2[3] = MA4_Data2[2];
  MA4_Data2[2] = MA4_Data2[1];
  MA4_Data2[1] = MA4_Data2[0];
  return MA4_Data2[0];
}
float Moving_Average3(float data) {
  MA3_Data[0] = (data + MA3_Data[1] + MA3_Data[2]) / 3;
  MA3_Data[2] = MA3_Data[1];
  MA3_Data[1] = MA3_Data[0];
  return MA3_Data[0];
}
float Moving_Average2(float data) {
  MA2_Data[0] = (data + MA2_Data[1]) / 2;
  MA2_Data[1] = MA2_Data[0];
  return MA2_Data[0];
}

void SERIAL_Print_ISR() { SERIAL_Print_Flag = 1; }
void IMU_ISR() { IMU_Flag = 1; }
void SERIAL_Print() {
  //   PC.printf(
  //   "1Pitch:%5.2f 1Roll:%5.2f 2Pitch:%5.2f 2Roll:%5.2f PenAngle:%5.2f \n\r",
  //   IMU1_Pitch, IMU1_Roll, IMU2_Pitch, IMU2_Roll, PEN_Angle);
  //   PC.printf("X: %5.2f, Y: %5.2f Z: %5.2f  X_A: %5.2f Y_A: %5.2f Z_A: %5.2f
  //   "
  //             "Pitch: %5.2f Roll: %5.2f \n\r",
  //             IMU1_X_Linear_Acc, IMU1_Y_Linear_Acc, IMU1_Z_Linear_Acc,
  //             IMU1_X_Acc, IMU1_Y_Acc, IMU1_Z_Acc, IMU1_Pitch, IMU1_Roll);

  //   PC.printf("xddot:%5.2f yddot:%5.2f ZDDOT:%5.2f YDDOT:%5.2f \n\r", x_ddot,
  //             y_ddot, Z_DDOT, Y_DDOT);

  //   PC.printf("%f %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f \n\r", t.read(),
  //             FD_Acc_u[0], FD_OutputAcc[0], FD_OutputVel[0],
  //             FD_OutputVel_Fil[0], FD_OutputPos[0], FD_OutputPos_Fil[0]);

//   PC.printf("%f %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f \n\r", t.read(),
//             FD_Acc_u[0], FD_OutputAcc[0], FD_OutputAcc_Fil[0], FD_OutputVel[0],
//             FD_OutputPos_Fil[0], FD_OutputVel_Unfiltered[0],
//             FD_OutputPos_Unfiltered[0],PEN_Angle);

  PC.printf("%f %5.2f %5.2f %5.2f %5.2f %5.2f \n\r", t.read(),
            FD_Acc_u[0],FD_OutputAcc_Fil[0], FD_OutputVel[0],
            FD_OutputPos_Fil[0],PEN_Angle);


  //   PC.printf("%f %5.2f %5.2f %5.2f %5.2f %5.2f \n\r", t.read(), FD_Acc_u[0],
  //             Z_DDOT_Fil5, Z_DDOT_Fil4, Z_DDOT_Fil3, Z_DDOT_Fil2);

  // PC.printf("Pen: %5.2f,Pitch: %5.2f, Roll: %5.2f
  // \n\r",PEN_Angle,IMU1_Pitch,IMU1_Roll);
  //  PC.printf("X: %5.2f, Y: %5.2f Z: %5.2f
  //  \n\r",imu1.lia.x,imu1.lia.y,imu1.lia.z);
  // PC.printf("X: %5.2f, Y: %5.2f Z: %5.2f
  // \n\r",imu1.euler.pitch,imu1.euler.roll,imu1.euler.yaw); PC.printf("%0d
  // %5.1f %5.1f %5.1f\r\n", imu1.calib, imu1.euler.roll,
  //           imu1.euler.pitch, imu1.euler.yaw);
}