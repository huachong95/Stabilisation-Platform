/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "platform/mbed_thread.h"


// Blinking rate in milliseconds
#define BLINKING_RATE_MS                                                    500
RawSerial PC(USBTX, USBRX, 115200); // tx, rx for CoolTerm output

int main()
{
    // Initialise the digital pin LED1 as an output
    DigitalOut led(LED1);
    AnalogIn CURRENT_Sensor(A1);

    while (true) {
        // led = !led;
        // thread_sleep_for(BLINKING_RATE_MS);
float current_temp=CURRENT_Sensor.read();
printf("current_temp: %f \n\r",current_temp);
thread_sleep_for(500);
    }
}
