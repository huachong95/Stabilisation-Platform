#include "mbed.h"
#include "motor.h"
#include "serial.h"
#include "sonar.h"
#include <iostream>
#include <stdio.h>

DigitalOut led1(LED1);  //Creates led 1 object using the in-built LED1 on STM32 microcontroller
DigitalOut led2(LED2);  //Creates led 2 object using the in-built LED2 on STM32 microcontroller
DigitalOut led3(D8);    //Creates led 3 object using external LED connected to Digital Pin 8
DigitalOut led4(D9);    //Creates led 4 object using external LED connected to Digital Pin 9
DigitalOut led5(D10);   //Creates led 5 object using external LED connected to Digital Pin 10
DigitalOut led6(D11);   //Creates led 6 object using external LED connected to Digital Pin 11
Thread LED_thread(osPriorityNormal);   //Creates object for LED thread

//Function declarations

void LED_Thread()
{
while(1){

    }
}

void LED_Init()
{
    LED_thread.start(LED_Thread);
}

void LED_Led1()
{
    led1=!led1; //Toggles led1
}
void LED_Led2()
{
    led2=!led2; //Toggles led2
}
void LED_Led3()
{
    led3=!led3; //Toggles led3
}
void LED_Led4()
{
    led4=!led4; //Toggles led4
}
void LED_Led5()
{
    led5=!led5; //Toggles led5
}
void LED_Led6()
{
    led6=!led6; //Toggles led6
}