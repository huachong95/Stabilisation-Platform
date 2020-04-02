/*
Refer to SRF08 Ultrasonic Sensor Characteristation Documentation for further information on SRF08 characteristics (Internal Innosparks Document)
For official documentation, use the SRF08 DataSheet
*/

#define SONAR1 0xE0     //I2C address of left sonar
#define SONAR2 0xE2     //I2C address of 1st front sonar
#define SONAR3 0xE4     //I2C address of 2nd front sonar
#define SONAR4 0xE6     //I2C address of right sonar
#define READY 1
#define IDLE 0
#define SONARFIREINTERVAL 0.06    //SONAR fire interval of 45ms
#define SONARREADINTERVAL 0.01    //SONAR delay interval of 10ms
#define SONARMAXRANGE 100   //Limits maximum SONAR effective range to 100cm
#define FRONTCRITICALDISTANCE 20
#define SIDECRITICALDISTANCE 10

#include "mbed.h"
#include "motor.h"
#include "serial.h"
#include "led.h"
#include <iostream>
#include <stdio.h>

I2C SONAR(PB_9, PB_8); //Creates I2C object for SONARs
Ticker SONAR_Fire_ISR; //Creates ticker timer interrupt for SONAR
Timeout SONAR_Read_ISR; //Creates timeout interrupt for SONAR, waiting for SONAR reply
Thread SONAR_thread(osPriorityNormal);

//Ultrasonic Sensor SRF08 Variables
char SONAR_Cmd[2];
char SONAR_ReadBuffer[2];
int SONAR_Range[4];
volatile bool SONAR_FireFlag=IDLE; //initialises flag for firing of SONAR sensors
volatile bool SONAR_ReadFlag=IDLE; //initialises flag for reading of SONAR sensors


//Function Declarations
void SONAR_FireISR();
void SONAR_ReadISR();
void SONAR_Fire(const int address1,const int address2,const int address3,const int address4);
void SONAR_Read(const int address1,const int address2,const int address3,const int address4);

void SONAR_Thread()
{
    while(1) {
        switch (SONAR_FireFlag) {
            case IDLE:
                break;
            case READY: //SONARs ready to fire
                SONAR_Fire(SONAR1,SONAR2,SONAR3,SONAR4); //SONAR_FireFlag triggers the firing of SONARs
                SONAR_FireFlag=IDLE; //sets the SONAR_FireFlag to IDLE
                break;
        }
        switch (SONAR_ReadFlag) {
            case IDLE:
                break;
            case READY: //SONARs info ready to be read
                SONAR_Read(SONAR1,SONAR2,SONAR3,SONAR4); //SONAR_ReadFlag triggers the reading of SONARS
                SONAR_ReadFlag=IDLE; //Resets the SONAR_ReadFlag to IDLE
                break;
        }
    }
}

void SONAR_Init()
{
    SONAR.frequency(400000);    //I2C frequency set to 400Kbps
    //SONAR settings/configuration can be obtained via SRF08 Datasheet
    //sets the maximum range of Ultrasonic Sensors
    // Sensor Range Setting= (1/4.3cm)*Range in cm [in Decimal]
    SONAR_Cmd[0]=0x02; // sets to range register at location 2
    SONAR_Cmd[1]=SONARMAXRANGE/4.3; //Max Range set to 1m
//    SONAR_Cmd[1]=0x8C; //Max Range set to 6m
//    SONAR_Cmd[1]=0xFF; //Max Range set to 11m
    SONAR.write(SONAR1, SONAR_Cmd, 2);
    SONAR.write(SONAR2, SONAR_Cmd, 2);
    SONAR.write(SONAR3, SONAR_Cmd, 2);
    SONAR.write(SONAR4, SONAR_Cmd, 2);

    //sets the analogue gain (sensitivity) of Ultrasonic Sensors
    SONAR_Cmd[0]=0x01; //sets to gain register at location 1
    SONAR_Cmd[1]=0x03; //sets analogue gain to 103
//    SONAR_Cmd[1]=0x1F; //sets analogue gain to 1025
    SONAR.write(SONAR1, SONAR_Cmd, 2);
    SONAR.write(SONAR2, SONAR_Cmd, 2);
    SONAR.write(SONAR3, SONAR_Cmd, 2);
    SONAR.write(SONAR4, SONAR_Cmd, 2);
    SONAR.write(SONAR4, SONAR_Cmd, 2);
    SONAR_Fire_ISR.attach(&SONAR_FireISR,SONARFIREINTERVAL); //Attaches ticker timer interrupt for SONAR firing
    SONAR_thread.start(SONAR_Thread);
}

void SONAR_FireISR()
{
    SONAR_FireFlag=READY;   //ISR to set SONAR_FireFlag state to ready
}
void SONAR_ReadISR()
{
    SONAR_ReadFlag=READY;   //ISR to set SONAR_ReadFlag state to ready
}

void SONAR_Fire(const int address1,const int address2,const int address3,const int address4)
{
    SONAR_Cmd[0] = 0x00; //set pointer reg to ‘SONAR_Cmd register'
    SONAR_Cmd[1] = 0x51; // config data byte1
    SONAR.write(address1, SONAR_Cmd, 2);
    SONAR.write(address2,SONAR_Cmd,2);
    SONAR.write(address3,SONAR_Cmd,2);
    SONAR.write(address4,SONAR_Cmd,2);
    SONAR_Read_ISR.attach(&SONAR_ReadISR,SONARREADINTERVAL); //attaches timeout interrupt to indicate US data ready to read
}
void SONAR_Read(const int address1,const int address2,const int address3,const int address4)
{
    SONAR_Cmd[0] = 0x02; //set pointer reg to 'data register'
    SONAR.write(address1, SONAR_Cmd, 1); //send to pointer 'read range'
    SONAR.read(address1, SONAR_ReadBuffer, 2); //read the two-byte range data
    SONAR_Range[0] = ((SONAR_ReadBuffer[0] << 8) + SONAR_ReadBuffer[1]);
    SONAR.write(address2, SONAR_Cmd, 1); //send to pointer 'read range'
    SONAR.read(address2, SONAR_ReadBuffer, 2); //read the two-byte range data
    SONAR_Range[1] = ((SONAR_ReadBuffer[0] << 8) + SONAR_ReadBuffer[1]);
    SONAR.write(address3, SONAR_Cmd, 1); //send to pointer 'read range'
    SONAR.read(address3, SONAR_ReadBuffer, 2); //read the two-byte range data
    SONAR_Range[2] = ((SONAR_ReadBuffer[0] << 8) + SONAR_ReadBuffer[1]);
    SONAR.write(address4, SONAR_Cmd, 1); //send to pointer 'read range'
    SONAR.read(address4, SONAR_ReadBuffer, 2); //read the two-byte range data
    SONAR_Range[3] = ((SONAR_ReadBuffer[0] << 8) + SONAR_ReadBuffer[1]);
    for (int i=0; i<4; i++) {
        if ((SONAR_Range[i]>SONARMAXRANGE)||(SONAR_Range[i]==0)) {
            SONAR_Range[i]=90000;   //if SONAR obstacles are detected further than the stated SONARMAXRANGE, an excessively large distance is sent to prevent interference with LIDAR scans
        }
    }
}
int SONAR_GetSONAR1()
{
    return SONAR_Range[0];  //returns Sonar 1 distance if requested by other threads
}
int SONAR_GetSONAR2()
{
    return SONAR_Range[1];  //returns Sonar 2 distance if requested by other threads
}
int SONAR_GetSONAR3()
{
    return SONAR_Range[2];  //returns Sonar 3 distance if requested by other threads
}
int SONAR_GetSONAR4()
{
    return SONAR_Range[3];  //returns Sonar 4 distance if requested by other threads
}

