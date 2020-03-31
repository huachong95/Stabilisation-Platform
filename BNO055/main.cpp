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

// Blinking rate in milliseconds
#define BLINKING_RATE_MS 500
RawSerial PC(USBTX, USBRX, 115200); // tx, rx for CoolTerm output
DigitalOut led(LED1);
BNO055 imu1(D14, D15);
Ticker ANGLE_ISR;
Ticker FD_Computation_ISR;
Ticker SERIAL_PRINT;
Timer t;

float IMU_Pitch = 0.0;
float IMU_Roll = 0.0;
float IMU_X_Acc = 0.0;
float IMU_Y_Acc = 0.0;
float IMU_Z_Acc = 0.0;
float IMU_X_Linear_Acc = 0.0;
float IMU_Y_Linear_Acc = 0.0;
float IMU_Z_Linear_Acc = 0.0;
bool SERIAL_Print_Flag = 0;
bool IMU_Flag = 0;
float PEN_Angle = 0.0;

float x_ddot = 0.0;
float y_ddot = 0.0;

float Y_DDOT = 0.0;
float Z_DDOT = 0.0;

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

const float A_filter_alpha = 0.975;
const float V_filter_alpha = 1;
const float P_filter_alpha = 1;
void IMU_Init();
void IMU_Angle();
void IMU_Acceleration();
void IMU_Linear_Acceleration();
void Acceleration_Computation();
void IMU_ISR();
void SERIAL_Print();
void SERIAL_Print_ISR();
void FD_Computation();
float map(float in, float inMin, float inMax, float outMin, float outMax);

int main() {
  t.start();
  PC.baud(115200);
  PC.printf("BNO055 Hello World\r\n\r\n");
  led = 1;
  // Reset the BNO055
  imu1.reset();
  IMU_Init();
  ANGLE_ISR.attach(&IMU_ISR, IMU_INTERVAL);
  SERIAL_PRINT.attach(&SERIAL_Print_ISR, SERIAL_PRINT_INTERVAL);
  FD_Computation_ISR.attach(&FD_Computation, SAMPLING_TIME);
  // Check that the BNO055 is connected and flash LED if not
  //   if (!imu1.check())
  //     while (true) {
  //       PC.printf("Waiting for IMU Connection \n\r");
  //       thread_sleep_for(100);
  //     }

  while (true) {
    imu1.setmode(OPERATION_MODE_NDOF);

    if (SERIAL_Print_Flag) {
      SERIAL_Print();
      SERIAL_Print_Flag = 0;
    }
    if (IMU_Flag) {
      IMU_Angle();
      //   IMU_Acceleration();
      IMU_Linear_Acceleration();
      Acceleration_Computation();
      FD_Computation();

      IMU_Flag = 0;
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
}

void IMU_Angle() {
  imu1.get_angles();
  IMU_Pitch = imu1.euler.pitch;
  IMU_Roll = imu1.euler.roll;
  PEN_Angle = sqrt((IMU_Pitch * IMU_Pitch) + (IMU_Roll * IMU_Roll));
}

void IMU_Acceleration() {
  imu1.get_accel();
  IMU_X_Acc = imu1.accel.x;
  IMU_Y_Acc = imu1.accel.y;
  IMU_Z_Acc = imu1.accel.z;
}

void IMU_Linear_Acceleration() {
  imu1.get_lia();
  IMU_X_Linear_Acc = imu1.lia.x;
  IMU_Y_Linear_Acc = imu1.lia.y;
  IMU_Z_Linear_Acc = imu1.lia.z;
}

void Acceleration_Computation() {
  Z_DDOT = IMU_Y_Linear_Acc * sin(IMU_Pitch) -
           IMU_X_Linear_Acc * sin(IMU_Roll) +
           IMU_Z_Linear_Acc * cos(IMU_Pitch) + IMU_Z_Linear_Acc * cos(IMU_Roll);

  y_ddot =
      IMU_Y_Linear_Acc * cos(IMU_Pitch) - IMU_Z_Linear_Acc * sin(IMU_Pitch);

  x_ddot = IMU_X_Linear_Acc * cos(IMU_Roll) + IMU_Z_Linear_Acc * sin(IMU_Roll);

  Y_DDOT = sqrt((x_ddot * x_ddot) + (y_ddot * y_ddot));
}
void FD_Computation() {
  FD_Acc_u[0] = Z_DDOT; // Updates new acceleration result
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

  FD_OutputVel_Unfiltered[0] =
      SAMPLING_TIME / 2 * (FD_OutputAcc[0] + FD_OutputAcc[1]) +
      FD_OutputVel_Unfiltered[1];
  //   FD_OutputVel_Fil[0] =
  //       V_filter_alpha *
  //       (FD_OutputVel_Fil[1] + FD_OutputVel[0] - FD_OutputVel[1]);
  FD_OutputPos_Unfiltered[0] =
      SAMPLING_TIME / 2 *
          (FD_OutputVel_Unfiltered[0] + FD_OutputVel_Unfiltered[1]) +
      FD_OutputPos_Unfiltered[1];
  //   FD_OutputPos[0] =
  //       SAMPLING_TIME / 2 * (FD_OutputVel_Fil[0] + FD_OutputVel_Fil[1]) +
  //       FD_OutputPos[1];
  //   FD_OutputPos_Fil[0] = P_filter_alpha * (FD_OutputPos_Fil[1] +
  //                                           FD_OutputPos[0] -
  //                                           FD_OutputPos[1]);
  FD_OutputAcc[1] = FD_OutputAcc[0];
  FD_OutputVel[1] = FD_OutputVel[0]; // Shifts the velocity results
  FD_OutputPos[1] = FD_OutputPos[0]; // Shifts the position results
  FD_OutputAcc_Fil[1] = FD_OutputAcc_Fil[0];
  FD_OutputVel_Fil[1] = FD_OutputVel_Fil[0];
  FD_OutputPos_Fil[1] = FD_OutputPos_Fil[0];
  FD_OutputVel_Unfiltered[1] = FD_OutputVel_Unfiltered[0];
  FD_OutputPos_Unfiltered[1] = FD_OutputPos_Unfiltered[0];
}
void SERIAL_Print_ISR() { SERIAL_Print_Flag = 1; }
void IMU_ISR() { IMU_Flag = 1; }
void SERIAL_Print() {
  //   PC.printf("X: %5.2f, Y: %5.2f Z: %5.2f  X_A: %5.2f Y_A: %5.2f Z_A: %5.2f
  //   "
  //             "Pitch: %5.2f Roll: %5.2f \n\r",
  //             IMU_X_Linear_Acc, IMU_Y_Linear_Acc, IMU_Z_Linear_Acc,
  //             IMU_X_Acc, IMU_Y_Acc, IMU_Z_Acc, IMU_Pitch, IMU_Roll);
  //   PC.printf("xddot:%5.2f yddot:%5.2f ZDDOT:%5.2f YDDOT:%5.2f \n\r", x_ddot,
  //             y_ddot, Z_DDOT, Y_DDOT);
  //   PC.printf("%f %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f \n\r", t.read(),
  //             FD_Acc_u[0], FD_OutputAcc[0], FD_OutputVel[0],
  //             FD_OutputVel_Fil[0], FD_OutputPos[0], FD_OutputPos_Fil[0]);
  PC.printf("%f %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f \n\r", t.read(),
            FD_Acc_u[0], FD_OutputAcc[0], FD_OutputAcc_Fil[0], FD_OutputVel[0],
            FD_OutputPos[0], FD_OutputVel_Unfiltered[0],
            FD_OutputPos_Unfiltered[0]);
  // PC.printf("Pen: %5.2f,Pitch: %5.2f, Roll: %5.2f
  // \n\r",PEN_Angle,IMU_Pitch,IMU_Roll);
  //  PC.printf("X: %5.2f, Y: %5.2f Z: %5.2f
  //  \n\r",imu1.lia.x,imu1.lia.y,imu1.lia.z);
  // PC.printf("X: %5.2f, Y: %5.2f Z: %5.2f
  // \n\r",imu1.euler.pitch,imu1.euler.roll,imu1.euler.yaw); PC.printf("%0d
  // %5.1f %5.1f %5.1f\r\n", imu1.calib, imu1.euler.roll,
  //           imu1.euler.pitch, imu1.euler.yaw);
}