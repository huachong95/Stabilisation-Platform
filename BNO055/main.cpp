/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */
#define IMU_INTERVAL 0.01 //100Hz
#define SERIAL_PRINT_INTERVAL 0.01

#include "BNO055.h"
#include "mbed.h"
#include "platform/mbed_thread.h"

// Blinking rate in milliseconds
#define BLINKING_RATE_MS 500
RawSerial PC(USBTX, USBRX, 115200); // tx, rx for CoolTerm output
DigitalOut led(LED1);
BNO055 imu1(D14, D15);
Ticker ANGLE_ISR;
Ticker SERIAL_PRINT;

float IMU_Pitch=0.0;
float IMU_Roll=0.0;
bool SERIAL_Print_Flag=0;
bool IMU_Angle_Flag=0;


void IMU_Angle();
void SERIAL_Print();
void SERIAL_Print_ISR();
float map(float in, float inMin, float inMax, float outMin, float outMax);

int main() {
  PC.baud(115200);
  PC.printf("BNO055 Hello World\r\n\r\n");
  led = 1;
  // Reset the BNO055
  imu1.reset();
  ANGLE_ISR.attach(&IMU_Angle,IMU_INTERVAL);
SERIAL_PRINT.attach(&SERIAL_Print_ISR, SERIAL_PRINT_INTERVAL);
  // Check that the BNO055 is connected and flash LED if not
  if (!imu1.check())
    while (true) {
      PC.printf("Waiting for IMU Connection \n\r");
      thread_sleep_for(100);
    }
  // Display sensor information
  PC.printf("BNO055 found\r\n\r\n");
  PC.printf("Chip          ID: %0d\r\n", imu1.ID.id);
  PC.printf("Accelerometer ID: %0d\r\n", imu1.ID.accel);
  PC.printf("Gyroscope     ID: %0d\r\n", imu1.ID.gyro);
  PC.printf("Magnetometer  ID: %0d\r\n\r\n", imu1.ID.mag);
  PC.printf("Firmware version v%d.%0d\r\n", imu1.ID.sw[0], imu1.ID.sw[1]);
  PC.printf("Bootloader version v%d\r\n\r\n", imu1.ID.bootload);
  // Display chip serial number
  for (int i = 0; i < 4; i++) {
    PC.printf("%0d.%0d.%0d.%0d\r\n", imu1.ID.serial[i * 4],
              imu1.ID.serial[i * 4 + 1], imu1.ID.serial[i * 4 + 2],
              imu1.ID.serial[i * 4 + 3]);
  }
  PC.printf("\r\n");
      imu1.setmode(OPERATION_MODE_NDOF);
  while (true) {

    // imu1.get_calib();
    // imu1.get_angles();
    // imu1.get_lia();

    // thread_sleep_for(100);
        if (SERIAL_Print_Flag) {
      SERIAL_Print();
      SERIAL_Print_Flag = 0;
    }
    if (IMU_Angle_Flag){
        IMU_Angle();
        IMU_Angle_Flag=0;
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
void IMU_Angle(){
        imu1.get_angles();
    IMU_Pitch=imu1.euler.pitch;
    IMU_Roll=imu1.euler.roll;

}
void SERIAL_Print_ISR() { SERIAL_Print_Flag = 1; }
void IMU_Angle_ISR(){IMU_Angle_Flag=1;}
void SERIAL_Print(){
        PC.printf("Pitch: %5.2f, Roll: %5.2f \n\r",IMU_Pitch,IMU_Roll);
// PC.printf("X: %5.2f, Y: %5.2f Z: %5.2f \n\r",imu1.lia.x,imu1.lia.y,imu1.lia.z);
// PC.printf("X: %5.2f, Y: %5.2f Z: %5.2f \n\r",imu1.euler.pitch,imu1.euler.roll,imu1.euler.yaw);
    // PC.printf("%0d %5.1f %5.1f %5.1f\r\n", imu1.calib, imu1.euler.roll,
    //           imu1.euler.pitch, imu1.euler.yaw);
}