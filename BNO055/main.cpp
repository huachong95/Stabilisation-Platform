#define SERIAL_PRINT_INTERVAL 0.01

#include "mbed.h"
#include <cstdio>
#include <iostream>

RawSerial PC(USBTX, USBRX, 115200); // tx, rx for CoolTerm output
DigitalOut led(LED1);


Ticker SERIAL_PRINT;
Timer TIME1;

// VARIABLE INSTANTIATION
// SERIAL Variables
char SERIAL_RXDataBuffer[128];       // Serial buffer for incoming serial data
volatile char SERIAL_RX_Counter = 0; // Serial counter used in seral buffer
volatile bool SERIAL_Read_Flag =
    0; // ISR Flag indicating serial input was received
volatile bool SERIAL_Print_Flag = 0;



// FUNCTION DECLARATIONS
void SERIAL_Read();
void SERIAL_Print();
void SERIAL_Print_ISR();
float map(float in, float inMin, float inMax, float outMin,
          float outMax) ;


int main() {
  PC.attach(&SERIAL_Read); // attaches interrupt upon serial input
                           //   JOYSTICK_ISR.attach(&JOYSTICK_ISR_Read, 0.005),
  
  TIME1.start(); // Startsthe TIME1 timer
    SERIAL_PRINT.attach(&SERIAL_Print_ISR, SERIAL_PRINT_INTERVAL);

  while (1) {
    if (SERIAL_Read_Flag) {
      SERIAL_Read_Flag = 0;  // Clears the serial_read flag
      SERIAL_RX_Counter = 0; // Resets the RX erial buffer counter
      //            char* payload = strtok(SERIAL_RXDataBuffer, ",");
      //            MOTOR_Speed=atoi(payload);
    //   switch (SERIAL_RXDataBuffer[0]) {

    //    }
    }
  
  }
}

void SERIAL_Read() {
  SERIAL_RXDataBuffer[SERIAL_RX_Counter] =
      PC.getc(); // Gets every new character and stores it in the RX serial
                 // buffer
  SERIAL_RX_Counter++; // Increments RX counter upon each byte storage
  if (SERIAL_RXDataBuffer[SERIAL_RX_Counter - 1] ==
      0x0D) { // Since /r = ENTER on keyboard = 0x0D(terminating byte in serial
    // commands), Serial_Read Flag is activated upon detection of 0x0D,
    // indicating data ready to be read
    SERIAL_Read_Flag = 1;
  }
}
void SERIAL_Print() {
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


void SERIAL_Print_ISR() { SERIAL_Print_Flag = 1; }
