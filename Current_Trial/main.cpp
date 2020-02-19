/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "platform/mbed_thread.h"

// Blinking rate in milliseconds
#define BLINKING_RATE_MS 500
RawSerial PC(USBTX, USBRX, 115200); // tx, rx for CoolTerm output

float map(float in, float inMin, float inMax, float outMin, float outMax);

int main() {
  // Initialise the digital pin LED1 as an output
  DigitalOut led(LED1);
  AnalogIn CURRENT_Sensor(A0);

  while (true) {
    // led = !led;
    // thread_sleep_for(BLINKING_RATE_MS);
    float current_temp = CURRENT_Sensor.read();
    float current_actual = map(current_temp, 0.0, 1.0, -25, 25);
    printf("current_temp: %f current_actual: %f \n\r", current_temp, current_actual);
    thread_sleep_for(500);
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