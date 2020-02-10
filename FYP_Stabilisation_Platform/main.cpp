#define L_EN_PIN D7
#define R_EN_PIN D8
#define L_PWM_PIN D6
#define R_PWM_PIN D9
#define JOYSTICK_PIN A0
#define L_SWITCH_PIN D12
#define RH_ENCODER_A_PIN D3  // right motor encoder A interrupt pin
#define RH_ENCODER_B_PIN D2  // right motor encoder B interrupt pin
#define ENCODER_INTERVAL 0.1 // Encoder read interval

#include "mbed.h"
#include <cstdio>
#include <iostream>

RawSerial PC(USBTX, USBRX, 115200); // tx, rx for CoolTerm output
DigitalOut led(LED1);
InterruptIn LSWITCH(L_SWITCH_PIN);
DigitalOut L_EN(L_EN_PIN);
DigitalOut R_EN(R_EN_PIN);
PwmOut L_PWM(L_PWM_PIN);
PwmOut R_PWM(R_PWM_PIN);
InterruptIn RH_ENCODER_A(RH_ENCODER_A_PIN);
DigitalIn RH_ENCODER_B(RH_ENCODER_B_PIN);
AnalogIn JOYSTICK_Y(JOYSTICK_PIN); // Analog input for Joystick Y Position

Ticker MOTOR_ISR;
Ticker SERIAL_PRINT;
Ticker JOYSTICK_ISR;    // Ticker interrupt for updating of joystick position
Ticker EncoderCheckISR; // Ticker interrupt for Encoder ISR
Timer TIME1;

// VARIABLE INSTANTIATION
// SERIAL Variables
char SERIAL_RXDataBuffer[128];       // Serial buffer for incoming serial data
volatile char SERIAL_RX_Counter = 0; // Serial counter used in seral buffer
volatile bool SERIAL_Read_Flag =
    0; // ISR Flag indicating serial input was received
volatile bool SERIAL_Print_Flag=0; 

// JOYSTICK Variables
volatile bool JOYSTICK_Read_Flag = 0;
float JOYSTICK_Y_Position = 0.0;

// LSWITCH Variables
bool LSWITCH_Flag = 0;

// MOTOR Variables
volatile bool MOTOR_Write_Flag = 0;
volatile int PWM_Mode = 1; // 1 == 25%, 2==50%, 3==75%, 4=100%
float MotorSpeed = 0.0;
float L_PWMSpeed = 0.0;
float R_PWMSpeed = 0.0;
float MAX_PWM = 1.0;             // Max of 1.0 (Full Power)
volatile long int ENCODER_Count = 0; // encoder ticks counter used in ISR
float TIME1_Current = 0.0;
float TIME1_Previous = 0.0;
float TIME1_Sample_Duration = 0.0;
float ENCODER_Wheel_Rev = 0.0;
float ENCODER_Change = 0.0;
int ENCODER_Speed = 0;
float ENCODER_Old_Count = 0.0;

// FUNCTION DECLARATIONS
void SERIAL_Read();
void SERIAL_Print();
void SetSpeed(int MotorSpeed);
void ENCODER_Check();
void ENCODER_Event();
void JOYSTICK_Read();
void LSWITCH_Home();
float map(float in, float inMin, float inMax, float outMin, float outMax);
void MOTOR_ISR_Write();
void JOYSTICK_ISR_Read();
void SERIAL_Print_ISR();
void LSWITCH_Rise_ISR();
void LSWITCH_Fall_ISR();

int main() {
  PC.attach(&SERIAL_Read); // attaches interrupt upon serial input
//   JOYSTICK_ISR.attach(&JOYSTICK_ISR_Read, 0.005),
      MOTOR_ISR.attach(&MOTOR_ISR_Write, 0.001);
  EncoderCheckISR.attach(&ENCODER_Check, ENCODER_INTERVAL);
  SERIAL_PRINT.attach(&SERIAL_Print_ISR, 1);

  L_PWM.period(0.00004);
  R_PWM.period(0.00004);
  RH_ENCODER_A.rise(&ENCODER_Event);
  RH_ENCODER_A.fall(&ENCODER_Event);
  LSWITCH.rise(&LSWITCH_Rise_ISR);
  LSWITCH.fall(&LSWITCH_Fall_ISR);
  TIME1.start(); // Startsthe TIME1 timer



  while (1) {
    if (SERIAL_Read_Flag) {
      SERIAL_Read_Flag = 0;  // Clears the serial_read flag
      SERIAL_RX_Counter = 0; // Resets the RX erial buffer counter
      //            char* payload = strtok(SERIAL_RXDataBuffer, ",");
      //            MotorSpeed=atoi(payload);
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
        char *footer = strtok(NULL, ",");                // Expects: '\r'
        MotorSpeed = atoi(payload);
        break;
      }
      case 'J': {
        JOYSTICK_ISR.attach(&JOYSTICK_ISR_Read, 0.005);
        break;
      }

      case 'S': {
        JOYSTICK_ISR
            .detach(); // Detach Interrupt for reading of Joystick position
        break;
      }

      case 'M': {
        MAX_PWM = (float)(100 * (SERIAL_RXDataBuffer[1] - '0') +
                          10 * (SERIAL_RXDataBuffer[2] - '0') +
                          SERIAL_RXDataBuffer[3] - '0') /
                  100;
        break;
      }
      }
    }
    if (MOTOR_Write_Flag) {
      SetSpeed(MotorSpeed);
      MOTOR_Write_Flag = 0;
    }

    if (JOYSTICK_Read_Flag) {
      JOYSTICK_Read();
      JOYSTICK_Read_Flag=0;
    }
    if (SERIAL_Print_Flag){
        SERIAL_Print();
        SERIAL_Print_Flag=0; 
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
  PC.printf("%f %f \n", TIME1_Current, MotorSpeed);
  //    printf("%f_%f \n",L_PWMSpeed,R_PWMSpeed);
  //   printf(" RSpeed: %f, ENCODER_Count: %ld \n\r", ENCODER_Wheel_Rev, ENCODER_Count);
  //    printf("ENCODER_Count: %f \n\r",ENCODER_Count);
  PC.printf("LSwitch State: %i \n\r", LSWITCH_Flag);
}

void SetSpeed(int MotorSpeed) {
  if (MotorSpeed >= 0) {
    R_EN = 1; // Enable motor to spin rightwards
    L_EN = 1; // Disable motor to spin leftwards
    //        R_PWMSpeed=map(abs(MotorSpeed),0,100,0,1.0);
    //        R_PWM.write(R_PWMSpeed);
    //        L_PWMSpeed=0;
    //        L_PWM.write(L_PWMSpeed);
    L_PWMSpeed = map(abs(MotorSpeed), 0, 100, 0, MAX_PWM);
    L_PWM.write(L_PWMSpeed);
    R_PWMSpeed = 0;
    R_PWM.write(R_PWMSpeed);
  } else {
    L_EN = 1; // Enable motor to spin leftwards
    R_EN = 1; // Disable motor to spin rightwards

    R_PWMSpeed = map(abs(MotorSpeed), 0, 100, 0, MAX_PWM);
    R_PWM.write(R_PWMSpeed);
    L_PWMSpeed = 0;
    L_PWM.write(L_PWMSpeed);
  }
}

void JOYSTICK_Read() {
  // float Xpos=1-XJoystick.read(); //inverts the horizontal joystick position
  JOYSTICK_Y_Position =
      1 - JOYSTICK_Y.read(); // inverts the vertical Joystick position
  MotorSpeed = map(JOYSTICK_Y_Position, 0.0, 1.0, -100,
                   100); // maps X position to pwm of motor
}

float map(float in, float inMin, float inMax, float outMin,
          float outMax) // use to map Joystick readings
{
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

// encoder event for the interrupt call
void ENCODER_Event() {
  if ((RH_ENCODER_A) == 1) {
    if ((RH_ENCODER_B) == 0) {
      ENCODER_Count++;
    } else {
      ENCODER_Count--;
    }
  } else {
    if ((RH_ENCODER_B) == 0) {
      ENCODER_Count--;
    } else {
      ENCODER_Count++;
    }
  }
}

void ENCODER_Check() {
  TIME1_Current = TIME1.read();
  TIME1_Sample_Duration = TIME1_Current - TIME1_Previous;
  TIME1_Previous = TIME1_Current;

  // since encoder feedback resolution is 17 for 1 revolution (shaft
  ENCODER_Change = ENCODER_Count - ENCODER_Old_Count;
  ENCODER_Wheel_Rev = ENCODER_Change / (17 * ENCODER_INTERVAL) * 60; // right wheel RPM
  //    ENCODER_Speed = ENCODER_Wheel_Rev * 2 * 3.1415 * 0.05; //velocity=r*w
  //    (radius of wheel is 5cm) ENCODER_Speed=60*ENCODER_Wheel_Rev;
  ENCODER_Old_Count = ENCODER_Count;
}

void LSWITCH_Home(){
    while(LSWITCH_Flag==0){
        SetSpeed(40);
    }
}

// ISR Functions
void JOYSTICK_ISR_Read() { JOYSTICK_Read_Flag = 1; }
void MOTOR_ISR_Write() { MOTOR_Write_Flag = 1; }
void LSWITCH_Rise_ISR() { LSWITCH_Flag = 0; } // LSWITCH is released
void LSWITCH_Fall_ISR() { LSWITCH_Flag = 1; } // LSWITCH is being pressed
void SERIAL_Print_ISR(){ SERIAL_Print_Flag=1;}