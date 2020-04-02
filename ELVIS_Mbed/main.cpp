#include "mbed.h"
#include "motor.h"
#include "led.h"
#include "serial.h"
#include "sonar.h"
#include <iostream>
#include <cstdio.h>


int main()
{
    MOTOR_Init();   //initialises motors and motor thread
    SERIAL_Init();  //initialises serial thread
    LED_Init();     //initialises led thread
    SONAR_Init();   //initialises sonar thread
    while(1) {
    }//WHILE
}//MAIN

/*To deactivate various threads, comment out the Init function. EG: //LED_Init().
*/








