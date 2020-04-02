#define TESTWAITINTERVAL 1  //Interval duration for wait interval used in system TEST functions
#define KEYPRESSTIMEOUT 10  //Interval timeout duration for system TEST functions
#define DATAPRINTINTERVAL 0.02 //Interval duration for serial printing 
#define BATTERYPRINTINTERVAL 60 //Interval duration for serial printing battery status
#define SYSTEMTIMEOUTINTERVAL 0.3 //Interval duration for system status timeout check

#include "mbed.h"
#include "motor.h"
#include "led.h"
#include "sonar.h"
#include <iostream>
#include <stdio.h>
#include <string.h>

RawSerial PC(USBTX, USBRX,115200);             //Creates serial communication object between the Mbed and PC (NUC/Raspberry Pi). RawSerial required for Mbed OS5
Ticker DataPrint_ISR;                          //Creates ticker timer object for serial printing of data
Ticker BatteryVoltagePrint_ISR;                //Creates a ticker timer object for serial printing battery voltage
//Timeout SERIAL_SystemStatus_ISR;               //Creates a ticker timer timeout object for checking system status
Thread SERIAL_thread(osPriorityNormal);        //Creates serial thread
Timer t;    //creates timer object used in system TEST functions

char SERIAL_RXDataBuffer[1024];     //Serial buffer for incoming serial data
volatile char RXcounter=0;          //Serial counter used in seral buffer
volatile bool SERIAL_ReadFlag=0;    //ISR Flag indicating serial input was received
volatile bool SERIAL_Print_Flag=0;  //ISR Flag indicating serial output ready to be printed
volatile bool SERIAL_Print_Battery_Flag=0; //ISR Flag indicating serial battery voltage output ready to be printed
int LeftSpeed;  //Placeholder for left speed (rpm) for left motor
int RightSpeed; //Placeholder for right speed (rpm) for right motor
bool TEST_Stop_Flag; //Flag terminating TEST functions, dependant on user input

//Function Declarations
void SERIAL_OnInput();
void SERIAL_SendData();
void SERIAL_PrintBatteryVoltage();
void SERIAL_ClearBuffer();
void SERIAL_MOTOR_Test();
void SERIAL_SONAR_Test();
void SERIAL_Keypress();
void SERIAL_SystemStatus();

void SERIAL_Thread()
{
    while(1) {
        if (SERIAL_ReadFlag) {
            SERIAL_ReadFlag=0;  //Clears the serial_read flag
            RXcounter=0;        //Resets the RX erial buffer counter
//            SERIAL_SystemStatus_ISR.attach(&SERIAL_SystemStatus, SYSTEMTIMEOUTINTERVAL); //attaches the timeout interrupt to stop robot motion after specifed interval with no serial input
            switch (SERIAL_RXDataBuffer[0]) {
                /*
                Checks the 1st byte in the RX serial buffer.
                ? --> DEFAULT motor command from ROS
                T --> Test sequence activated
                */
                case '?':
                    /*
                    Data Packet Example: ?,50_-50,\r
                    A complete data packet requires it to start with a "?" and terminate with a "/r" (aka ENTER on Keyboard)
                    Left motor speed: 50rpm, right motor speed: -50rpm
                    */
                    char* header = strtok(SERIAL_RXDataBuffer, ",");   // Expects: '?'
                    char* payload = strtok(NULL, ",");  // Expects: <payload>
                    char* footer = strtok(NULL, ",");   // Expects: '\r'
                    LeftSpeed = atoi(strtok(payload, "_"));
                    RightSpeed = atoi(strtok(NULL, "_"));
                    MOTOR_SetSpeed(LeftSpeed,RightSpeed); //Sends the motor speeds feedback to MOTOR Thread
                    break;

                case 'T':
                    /*
                    Packet Example: TA
                    */
                    TEST_Stop_Flag=0;  //Clears TEST_Stop_Flag
                    if (SERIAL_RXDataBuffer[1] == 'A') { //ALL test functions to be activated
                        SERIAL_MOTOR_Test();
                        SERIAL_SONAR_Test();
                    }
                    if (SERIAL_RXDataBuffer[1] =='M') {  //MOTOR test function activated
                        SERIAL_MOTOR_Test();
                    }
                    if (SERIAL_RXDataBuffer[1] =='S') {  //SONAR test function activated
                        SERIAL_SONAR_Test();
                    }
            }
        }
        if (SERIAL_Print_Flag) {
            //Example Data Print Packet: ?,50,-50,11500,11500,20,32,56,60,\r
            //LMotorSpeed = 50rpm, RMotorSpeed=-50rpm, LMotorPostionCounts: 11500, RMotorPositionCounts: 11500, Sonar1Distance= 20cm, Sonar2Distance=32cm, Sonar3Distance=56cm, Sonar4Distance=60cm
            printf("?H,%i_%i_%li_%li_%i_%i_%i_%i,\r",MOTOR_GetLSpeed(),MOTOR_GetRSpeed(),MOTOR_GetLPosition(),MOTOR_GetRPosition(),SONAR_GetSONAR1(),SONAR_GetSONAR2(),SONAR_GetSONAR3(),SONAR_GetSONAR4());
            SERIAL_Print_Flag=0; //Clears the serial print flag
        }
        if (SERIAL_Print_Battery_Flag) {
            printf("?V,%i,\r", MOTOR_GetBatteryVoltage());
            SERIAL_Print_Battery_Flag=0; //Clears the serial print battery flag
        }
    }
}
void SERIAL_Init()
{
    PC.attach(&SERIAL_OnInput); //attaches interrupt upon serial input
    DataPrint_ISR.attach(&SERIAL_SendData,DATAPRINTINTERVAL);  //attaches ticker timer interrupt. ISR runs every 20ms (50Hz)
    BatteryVoltagePrint_ISR.attach(&SERIAL_PrintBatteryVoltage,BATTERYPRINTINTERVAL); //attaches ticker timer interrupt to print battery voltage status every 60s
    SERIAL_thread.start(SERIAL_Thread);         //Starts the Serial thread
}

void SERIAL_OnInput()
{
    SERIAL_RXDataBuffer[RXcounter]= PC.getc();  //Gets every new character and stores it in the RX serial buffer
    RXcounter++;    //Increments RX counter upon each byte storage
    if (SERIAL_RXDataBuffer[RXcounter-1]==0x0D) {   //Since /r = ENTER on keyboard = 0x0D(terminating byte in serial commands), Serial_Read Flag is activated upon detection of 0x0D, indicating data ready to be read
        SERIAL_ReadFlag=1;
    }
}

void SERIAL_SendData()
{
    SERIAL_Print_Flag=1; //ISR which activates Serial_Print_Flag upon every ticker timer interrupt
}
void SERIAL_PrintBatteryVoltage()
{
    SERIAL_Print_Battery_Flag=1; //ISR which prints battery status upon every ticker timer interrupt
}
void SERIAL_ClearBuffer()
{
    for (int j=0; j<RXcounter; j++) {
        SERIAL_RXDataBuffer[j]=0; //clears the SERIAL_RXDataBuffer
    }
    RXcounter=0; //Resets RX Counter
}

void SERIAL_MOTOR_Test()    //MOTOR TEST Function
{
    printf("---STARTING THE MOTOR TEST SEQUENCE--- \n\r");
    wait(TESTWAITINTERVAL);
    printf("PLEASE PLACE THE LOGISTICS CART OFF THE GROUND \n\r");
    wait (TESTWAITINTERVAL);
    printf("PRESS ENTER WHEN READY. PRESS Q TO STOP TEST\n\r");
    SERIAL_Keypress();  //waits for user input to proceed with test
    if (TEST_Stop_Flag) {   //Terminates test if termination command is detected
        printf("MOTOR TEST SEQUENCE STOPPED \n\r");
        return;
    }
    MOTOR_Test();   //Motor commands sent to MOTOR Thread
    printf("MOTOR TEST COMPLETED. PRESS ENTER TO CONTINUE \n\r");
    SERIAL_Keypress();
}

void SERIAL_SONAR_Test()
{
    printf("---STARTING THE SONAR TEST SEQUENCE--- \n\r");
    wait(TESTWAITINTERVAL);
//TESTING OF SONAR 1
    printf("PLACE AN OBJECT IN FRONT OF SONAR 1 \n\r");
    wait (TESTWAITINTERVAL);
    printf("PRESS ENTER WHEN READY. PRESS Q TO STOP TEST\n\r");
    SERIAL_Keypress();  //waits for user input to proceed with test
    if (TEST_Stop_Flag) {   //Terminates test if termination command is detected
        printf("SONAR TEST SEQUENCE STOPPED \n\r");
        return;
    }
    printf("OBJECT DISTANCE FROM SONAR 1: %iCM\n\r", SONAR_GetSONAR1()); //obtains object distance from SONAR thread
    wait (TESTWAITINTERVAL);

//TESTING OF SONAR 2
    printf("PLACE AN OBJECT IN FRONT OF SONAR 2 \n\r");
    wait (TESTWAITINTERVAL);
    printf("PRESS ENTER WHEN READY. PRESS Q TO STOP TEST\n\r");
    SERIAL_Keypress();
    if (TEST_Stop_Flag) {
        printf("SONAR TEST SEQUENCE STOPPED \n\r");
        return;
    }
    printf("OBJECT DISTANCE FROM SONAR 2: %iCM\n\r", SONAR_GetSONAR2());
    wait (TESTWAITINTERVAL);

    //TESTING OF SONAR 3
    printf("PLACE AN OBJECT IN FRONT OF SONAR 3 \n\r");
    wait (TESTWAITINTERVAL);
    printf("PRESS ENTER WHEN READY. PRESS Q TO STOP TEST\n\r");
    SERIAL_Keypress();
    if (TEST_Stop_Flag) {
        printf("SONAR TEST SEQUENCE STOPPED \n\r");
        return;
    }
    printf("OBJECT DISTANCE FROM SONAR 3: %iCM\n\r", SONAR_GetSONAR3());
    wait (TESTWAITINTERVAL);

//TESTING OF SONAR 4
    printf("PLACE AN OBJECT IN FRONT OF SONAR 4 \n\r");
    wait (TESTWAITINTERVAL);
    printf("PRESS ENTER WHEN READY. PRESS Q TO STOP TEST\n\r");
    SERIAL_Keypress();
    if (TEST_Stop_Flag) {
        printf("SONAR TEST SEQUENCE STOPPED \n\r");
        return;
    }
    printf("OBJECT DISTANCE FROM SONAR 4: %iCM\n\r", SONAR_GetSONAR4());
    wait (TESTWAITINTERVAL);
    printf("SONAR TEST COMPLETED. PRESS ENTER TO CONTINUE \n\r");
    SERIAL_Keypress();
}

void SERIAL_Keypress()
{
    SERIAL_ClearBuffer();
    t.reset();
    t.start();  //starts the timer t
    while (t.read()<KEYPRESSTIMEOUT) { //waits for user input up to specified timeout

        if (SERIAL_RXDataBuffer[0]=='Q') {
            TEST_Stop_Flag=1; //sets the stop test flag to high to terminate test
            SERIAL_ClearBuffer();
            return;
        }

        else if (SERIAL_RXDataBuffer[0]==0x0D) {    //user is ready to proceed with test
            SERIAL_ClearBuffer();
            return;
        }
    }
    printf("NO INPUT FROM USER WAS PROVIDED. SYSTEM WILL PROCEED ON\n\r");
    wait(TESTWAITINTERVAL);
}

void SERIAL_SystemStatus()
{
// Stops ELVIS if no command has been received
    LeftSpeed = 0;
    RightSpeed =0;
    MOTOR_SetSpeed(LeftSpeed,RightSpeed); //Sends the motor speeds feedback to MOTOR Thread

}