#define ENCODER_CPR 30 // Encoder Pulses per revolution
#define SERIAL_PRINT_INTERVAL 0.01
#define SYSTEMTIMEOUTINTERVAL 0.1

#include "CASCADE.h"
#include "PID.h"
#include "mbed.h"
#include <cstdio>
#include <iostream>


RawSerial PC(USBTX, USBRX, 115200); // tx, rx for CoolTerm output
Ticker SERIAL_Print_TISR;
Ticker SERIAL_SystemStatus_ISR;
Timer TIME2;
Thread SERIAL_thread(osPriorityNormal); // Creates MOTOR thread

// VARIABLE INSTANTIATION
// SERIAL Variables
char SERIAL_RXDataBuffer[128];       // Serial buffer for incoming serial data
volatile char SERIAL_RX_Counter = 0; // Serial counter used in seral buffer
volatile bool SERIAL_Read_Flag =
    0; // ISR Flag indicating serial input was received
volatile bool SERIAL_Print_Flag = 0;

float SMOTOR_Speed = 0.0;
float SDEMANDED_Current = 0.0;
float SDEMANDED_Velocity = 0.0;
float SDEMANDED_Position = 0.0;
int SCASCADE_Mode=1;

// FUNCTION DECLARATIONS
void SERIAL_Read();
void SERIAL_Print();
void SERIAL_SystemStatus();
void SERIAL_Print_ISR();

void SERIAL_Thread() {
    TIME2.start();
  PC.attach(&SERIAL_Read); // attaches interrupt upon serial input
  SERIAL_Print_TISR.attach(&SERIAL_Print_ISR, SERIAL_PRINT_INTERVAL);
  SCASCADE_Mode = CASCADE_Get_CASCADE_Mode();
  while (1) {
    if (SERIAL_Read_Flag) {
      SERIAL_SystemStatus_ISR.attach(&SERIAL_SystemStatus,
                                     SYSTEMTIMEOUTINTERVAL);
      SERIAL_Read_Flag = 0;  // Clears the serial_read flag
      SERIAL_RX_Counter = 0; // Resets the RX erial buffer counter

      switch (SERIAL_RXDataBuffer[0]) {
      case '?': {
        /*
        Data Packet Example: ?,50_-50,\r
        A complete data packet requires it to start with a "?" and
        terminate with a "/r" (aka ENTER on Keyboard) Left motor speed:
        50rpm, right motor speed: -50rpm
        */
        char *header = strtok(SERIAL_RXDataBuffer, ","); // Expects: '?'
        char *payload = strtok(NULL, ",");               // Expects:<payload>
        char *footer = strtok(NULL, "\r");               // Expects: '\r'
        SMOTOR_Speed = -atoi(payload);
        break;
      }

      case 'P': {
        // Expects "P,150,\r"
        char *header = strtok(SERIAL_RXDataBuffer, ","); // Expects: '?'
        char *payload = strtok(NULL, ",");               // Expects:<payload>
        char *footer = strtok(NULL, "\r");               // Expects: '\r'
        SDEMANDED_Velocity = atoi(payload);
        break;
      }
      case 'V': {
        // Expects "V,2000,\r"
        char *header = strtok(SERIAL_RXDataBuffer, ","); // Expects: '?'
        char *payload = strtok(NULL, ",");               // Expects:<payload>
        char *footer = strtok(NULL, "\r");               // Expects: '\r'
        SDEMANDED_Velocity = atoi(payload);
        break;
      }

      case 'C': {
        char *header = strtok(SERIAL_RXDataBuffer, ","); // Expects: '?'
        char *payload = strtok(NULL, ",");               // Expects:<payload>
        char *footer = strtok(NULL, "\r");               // Expects: '\r'
        SDEMANDED_Current = (float)(atoi(payload)) / 100;
        break;
      }
      case 'A': {
        char *header = strtok(SERIAL_RXDataBuffer, ","); // Expects: '?'
        char *payload1 = strtok(NULL, ",");              // Expects:<payload>
        char *payload2 = strtok(NULL, ",");              // Expects:<payload>
        char *footer = strtok(NULL, "\r");               // Expects: '\r'
        SDEMANDED_Current = (float)(atoi(payload1)) / 100;
        SDEMANDED_Velocity = (float)(atoi(payload2));
        break;
      }
      case 'B': {
        char *header = strtok(SERIAL_RXDataBuffer, ","); // Expects: '?'
        char *payload1 = strtok(NULL, ",");              // Expects:<payload>
        char *payload2 = strtok(NULL, ",");              // Expects:<payload>
        char *payload3 = strtok(NULL, ",");              // Expects:<payload>
        char *footer = strtok(NULL, "\r");               // Expects: '\r'
        SDEMANDED_Current = (float)(atoi(payload1)) / 100;
        SDEMANDED_Velocity = (float)(atoi(payload2));
        SDEMANDED_Velocity = (float)(atoi(payload3));
        break;
      }
      }
    }

    if (SERIAL_Print_Flag) {
      SERIAL_Print();
      SERIAL_Print_Flag = 0;
    }
  }
}

void SERIAL_Init() { SERIAL_thread.start(SERIAL_Thread); }
void SERIAL_Read() {
  SERIAL_RXDataBuffer[SERIAL_RX_Counter] =
      PC.getc(); // Gets every new character and stores it in the RX serial
                 // buffer
  SERIAL_RX_Counter++; // Increments RX counter upon each byte storage
  if (SERIAL_RXDataBuffer[SERIAL_RX_Counter - 1] ==
      0x0D) { // Since /r = ENTER on keyboard = 0x0D(terminating byte in
              // serial
    // commands), Serial_Read Flag is activated upon detection of 0x0D,
    // indicating data ready to be read
    SERIAL_Read_Flag = 1;
  }
}
void SERIAL_Print() {
// PC.printf("Hello \n\r");
    if (SCASCADE_Mode == 1) {
      PC.printf("%f %f %f \n\r", TIME2.read(), CASCADE_Get_DEMANDED_Current(),
      CASCADE_Get_MOTOR_Current());
    } else if (SCASCADE_Mode == 2) {
      PC.printf("%f %f %f %f %f \n\r", TIME2.read(),
      CASCADE_Get_DEMANDED_Current_Total(),
                CASCADE_Get_MOTOR_Current(), CASCADE_Get_DEMANDED_Velocity(), CASCADE_Get_ENCODER_RPM());
    } else if (SCASCADE_Mode == 3) {
      PC.printf("%f %f %f %f %f %f %f \n\r", TIME2.read(),
                CASCADE_Get_DEMANDED_Current_Total(), CASCADE_Get_MOTOR_Current(),
                CASCADE_Get_DEMANDED_Velocity_Total(), CASCADE_Get_ENCODER_RPM(), CASCADE_Get_DEMANDED_Position(),
                CASCADE_Get_LEADSCREW_Pos());
    }
}

void SERIAL_SystemStatus() {
  SDEMANDED_Current = 0;
  SDEMANDED_Velocity = 0;
  //   SDEMANDED_Velocity = LEADSCREW_Position;
}

// ISR Functions
void SERIAL_Print_ISR() { SERIAL_Print_Flag = 1; }
