#define PI 3.141592654
#define L_EN_PIN D10
#define R_EN_PIN D8
#define L_PWM_PIN D6
#define R_PWM_PIN D9
#define JOYSTICK_PIN A5
#define L_SWITCH_PIN D12
#define RH_ENCODER_A_PIN D3        // right motor encoder A interrupt pin
#define RH_ENCODER_B_PIN D2        // right motor encoder B interrupt pin
#define CURRENT_SENSOR_PIN A0      // ACS712 Current Sensor pin
#define ENCODER_INTERVAL 0.01      // Encoder read interval
#define LSWITCH_SLEEP_DURATION 600 // Minimum cycle switch duration required
#define LEADSCREW_LEAD 8           // Lead in mm
#define LEADSCREW_MAX_RANGE 330
#define MAX_MOTORSPEED 4500
#define ENCODER_CPR 30 // Encoder Pulses per revolution
#define SERIAL_PRINT_INTERVAL 0.02
#define SYSTEMTIMEOUTINTERVAL 0.2
#define MOTOR_WRITE_RATE 0.01     // Write Rate of Motor
#define PID_POSITION_RATE 0.01    // 100Hz Sample Rate of PID_Position
#define PID_VELOCITY_RATE 0.001   // 1000HzSample Rate of PID_Velocity
#define PID_CURRENT_RATE 0.0001   // 10000HzSample Rate of PID_Current
#define CURRENT_MAX_RANGE 20      // Max Amps supported by Current Sensor
#define LEADSCREW_INITIAL_POS 150 // Leadscrew initial position
#define CASCADE_MODE 3            // 1==C, 2==C&V 3==C&V&P
#define IMU_INTERVAL 0.01         // 100Hz
#define HARDWARE_PMAX 250
#define HARDWARE_PMIN 70
// #define HARDWARE_VMAX 2500
// #define HARDWARE_VMIN -2500
#define HARDWARE_VMAX 6500
#define HARDWARE_VMIN 6500
#define HARDWARE_CMAX 15
#define HARDWARE_CMIN -15

#include "BNO055.h"
#include "PID.h"
#include "mbed.h"
#include <cstdio>
#include <iostream>

RawSerial PC(USBTX, USBRX, 115200); // tx, rx for CoolTerm output
BNO055 imu1(D14, D15);
BNO055 imu2(D5, D7);
DigitalOut led(LED1);
InterruptIn LSWITCH(L_SWITCH_PIN);
DigitalOut L_EN(L_EN_PIN);
DigitalOut R_EN(R_EN_PIN);
PwmOut L_PWM(L_PWM_PIN);
PwmOut R_PWM(R_PWM_PIN);
InterruptIn RH_ENCODER_A(RH_ENCODER_A_PIN);
DigitalIn RH_ENCODER_B(RH_ENCODER_B_PIN);
AnalogIn JOYSTICK_Y(JOYSTICK_PIN); // Analog input for Joystick Y Position
AnalogIn CURRENT_Sensor(CURRENT_SENSOR_PIN);
// PID PID_Position(9, 10.0, 0.0, PID_POSITION_RATE);
// // PID PID_Position(9, 0, 0.0, PID_POSITION_RATE);
// // PID PID_Velocity(0.55, 15.0, 0.0, PID_VELOCITY_RATE);
// PID PID_Velocity(0.5, 45.0, 0.0, PID_VELOCITY_RATE);
// PID PID_Current(65, 30.0, 0, PID_CURRENT_RATE);
// // PID PID_Current(65, 30.0, 0, PID_CURRENT_RATE);
PID PID_Position(10.6, 705.0, 0.0, PID_POSITION_RATE);
PID PID_Velocity(1.65, 60.0, 0.0, PID_VELOCITY_RATE);
PID PID_Current(40, 30.0, 0, PID_CURRENT_RATE);

Ticker ANGLE_ISR;
Ticker MOTOR_TISR;
Ticker SERIAL_Print_TISR;
Ticker SERIAL_SystemStatus_ISRR;
Ticker JOYSTICK_TISR;      // Ticker interrupt for updating of joystick position
Ticker ENCODER_Check_TISR; // Ticker interrupt for Encoder ISR
Ticker CURRENT_Sensor_TISR;
Ticker PID_Position_TISR;
Ticker PID_Velocity_TISR;
Ticker PID_Current_TISR;
Timer TIME1;

// VARIABLE INSTANTIATION

// IMU Variables
float IMU1_Pitch = 0.0;
float IMU1_Roll = 0.0;
float IMU1_X_Acc = 0.0;
float IMU1_Y_Acc = 0.0;
float IMU1_Z_Acc = 0.0;
float IMU1_X_Linear_Acc = 0.0;
float IMU1_Y_Linear_Acc = 0.0;
float IMU1_Z_Linear_Acc = 0.0;

float IMU2_Pitch = 0.0;
float IMU2_Roll = 0.0;
float IMU2_X_Acc = 0.0;
float IMU2_Y_Acc = 0.0;
float IMU2_Z_Acc = 0.0;
float IMU2_X_Linear_Acc = 0.0;
float IMU2_Y_Linear_Acc = 0.0;
float IMU2_Z_Linear_Acc = 0.0;

bool IMU_Flag = 0;
bool SERIAL_SystemStatus_Flag = 0;
float PEN_Angle = 0.0;

float x_ddot = 0.0;
float y_ddot = 0.0;

float Y_DDOT = 0.0;
float Z_DDOT = 0.0;
float Z_DDOT_Fil5 = 0.0;
float Z_DDOT_Fil4 = 0.0;
float Z_DDOT_Fil3 = 0.0;
float Z_DDOT_Fil2 = 0.0;
float Y_DDOT_Fil4 = 0.0;
float IMU1_Y_Filtered = 0.0;
float IMU1_X_Filtered = 0.0;
float IMU1_Z_Filtered = 0.0;

float LENGTH_Acc = 0.0;
// Finite Difference Implementation Variables
float FD_Acc_y[4] = {0, 0, 0, 0};
float FD_Acc_u[4] = {0, 0, 0, 0};
float FD_OutputAcc[2] = {0, 0};
float FD_OutputVel[2] = {0, 0};
float FD_OutputPos[2] = {0, 0}; // NEED TO CHANGE THIS TO INITIAL POSITION
float FD_OutputAcc_Fil[2] = {0, 0};
float FD_OutputVel_Fil[2] = {0, 0};
float FD_OutputPos_Fil[2] = {0, 0};
float FD_OutputVel_Unfiltered[2] = {0, 0};
float FD_OutputPos_Unfiltered[2] = {0, 0};
float MA5_Data[5] = {0, 0, 0, 0, 0};
float MA4_Data1[4] = {0, 0, 0, 0};
float MA4_Data2[4] = {0, 0, 0, 0};
float MA4_Data3[4] = {0, 0, 0, 0};
float MA3_Data[3] = {0, 0, 0};
float MA2_Data[2] = {0, 0};

const float A_filter_alpha = 0.975;
const float V_filter_alpha = 1;
const float P_filter_alpha = 0.99;

// SERIAL Variables
char SERIAL_RXDataBuffer[128];       // Serial buffer for incoming serial data
volatile char SERIAL_RX_Counter = 0; // Serial counter used in seral buffer
volatile bool SERIAL_Read_Flag =
    0; // ISR Flag indicating serial input was received
volatile bool SERIAL_Print_Flag = 0;

// JOYSTICK Variables
volatile bool JOYSTICK_Read_Flag = 0;
float JOYSTICK_Y_Position = 0.0;

// LSWITCH Variables
volatile bool LSWITCH_Flag = 0;
volatile bool LSWITCH_Complete_Home = 0;

// CURRENT_Sensor Variables
volatile bool CURRENT_Sensor_Flag = 0;
float CURRENT_Sensor_ADC_Reading = 0.5;
float MOTOR_Current = 0.0;
float MOTOR_Current_Raw = 0.0;
float CURRENT_Offset = 0.0;
const float CURRENT_Filter_Alpha = 0.15;
double CURRENT_Filter_Data[] = {0, 0};

// MOTOR Variables
volatile bool MOTOR_Write_Flag = 0;
volatile float MOTOR_Speed = 0;
float L_PWMSpeed = 0.0;
float R_PWMSpeed = 0.0;
float MAX_PWM = 1.0;            // Max of 1.0 (Full Power)
volatile int ENCODER_Count = 0; // encoder ticks counter used in ISR
float TIME1_Current = 0.0;
float TIME1_Previous = 0.0;
float TIME1_Sample_Duration = 0.0;
float ENCODER_RPM = 0.0;
float ENCODER_RPM_Raw = 0.0;
float ENCODER_Change = 0.0;
int ENCODER_Speed = 0;
float ENCODER_Old_Count = 0.0;

int Cascade_Mode = CASCADE_MODE;
bool LEADSCREW_Initialisation = 0;
bool PID_POSITION_INITIALISED = 0;
bool PID_VELOCITY_INITIALISED = 0;
bool PID_CURRENT_INITIALISED = 0;
bool PID_Position_Flag = 0;
bool PID_Velocity_Flag = 0;
bool PID_Current_Flag = 0;
float ERROR_Vel = 0.0;
float ERROR_Pos = 0.0;

// LEADSCREW Variables
float LEADSCREW_Position = 0.0;
float DEMANDED_Position = LEADSCREW_INITIAL_POS;
float DEMANDED_Velocity = 0;
float DEMANDED_Velocity_Total = 0;
float DEMANDED_Current = 0.0;
float DEMANDED_Current_Total = 0.0;
float MOTOR_Speed_PID = 0.0;

// FUNCTION DECLARATIONS
// IMU FUNCTIONS
void IMU_Init();
void IMU1_Angle();
void IMU2_Angle();
void IMU1_Acceleration();
void IMU2_Acceleration();
void IMU1_Linear_Acceleration();
void IMU2_Linear_Acceleration();
void Acceleration_Computation();
void IMU_ISR();
void SERIAL_SystemStatus_ISR();
void FD_Computation();
float Moving_Average5(float data);
float Moving_Average4_1(float data);
float Moving_Average4_2(float data);
float Moving_Average4_3(float data);
float Moving_Average3(float data);
float Moving_Average2(float data);

void SERIAL_Read();
void SERIAL_Print();
void SERIAL_SystemStatus();
void SetSpeed(int MOTOR_Speed);
void ENCODER_Check();
void ENCODER_Event();
void JOYSTICK_Read();
void LSWITCH_Home();
void CURRENT_Sensor_Read();
float CURRENT_Sensor_Offset();
float map(float in, float inMin, float inMax, float outMin, float outMax);
void MOTOR_ISR_Write();
void JOYSTICK_ISR_Read();
void SERIAL_Print_ISR();
void LSWITCH_Rise_ISR();
void LSWITCH_Fall_ISR();
void PID_Position_ISR();
void PID_Velocity_ISR();
void PID_Current_ISR();
void CURRENT_SENSOR_ISR_Read();

void PID_Position_Initialisation();
void PID_Velocity_Initialisation();
void PID_Current_Initialisation();
void PID_Position_Computation();
void PID_Velocity_Computation();
void PID_Current_Computation();
void Cascade_Initialisation(int Cascade_Mode);

int main() {
  PC.attach(&SERIAL_Read); // attaches interrupt upon serial input
  IMU_Init();
  ANGLE_ISR.attach(&IMU_ISR, IMU_INTERVAL);
  MOTOR_TISR.attach(&MOTOR_ISR_Write, MOTOR_WRITE_RATE);
  ENCODER_Check_TISR.attach(&ENCODER_Check, ENCODER_INTERVAL);
  L_PWM.period(0.00008);
  R_PWM.period(0.00008);
  //   L_PWM.period(0.00012);
  //   R_PWM.period(0.00012);
  LSWITCH.rise(&LSWITCH_Rise_ISR);
  LSWITCH.fall(&LSWITCH_Fall_ISR);
  TIME1.start(); // Startsthe TIME1 timer
  LSWITCH_Home();
  RH_ENCODER_A.rise(&ENCODER_Event);
  RH_ENCODER_A.fall(&ENCODER_Event);

  CURRENT_Sensor_TISR.attach(&CURRENT_SENSOR_ISR_Read, PID_CURRENT_RATE);
  CURRENT_Offset = CURRENT_Sensor_Offset(); // obtains the zero-offset current

  PID_Position_Initialisation();
  Cascade_Initialisation(CASCADE_MODE);
  PID_Position_Initialisation();
  SERIAL_Print_TISR.attach(&SERIAL_Print_ISR, SERIAL_PRINT_INTERVAL);

  while (1) {
    while (LEADSCREW_Initialisation == 0) {
      while (LEADSCREW_Position < LEADSCREW_INITIAL_POS) {
        SetSpeed(-35);
      }
      if (PID_Position_Flag) {
        PID_Position_Computation();
        SetSpeed(ERROR_Pos);
        PID_Position_Flag = 0;
      }
      if ((LEADSCREW_Position >= LEADSCREW_INITIAL_POS - 15) &&
          ((LEADSCREW_Position <= LEADSCREW_INITIAL_POS + 15))) {
        SetSpeed(0);
        LEADSCREW_Initialisation = 1; // Leadscrew Initialisation complete
      }
    }
    if (SERIAL_Read_Flag) {
      SERIAL_Read_Flag = 0;  // Clears the serial_read flag
      SERIAL_RX_Counter = 0; // Resets the RX erial buffer counter
      //            char* payload = strtok(SERIAL_RXDataBuffer, ",");
      //            MOTOR_Speed=atoi(payload);
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
        MOTOR_Speed = -atoi(payload);
        break;
      }

      case 'P': {
        // Expects "P,150,\r"
        char *header = strtok(SERIAL_RXDataBuffer, ","); // Expects: '?'
        char *payload = strtok(NULL, ",");               // Expects:<payload>
        char *footer = strtok(NULL, "\r");               // Expects: '\r'
        DEMANDED_Position = atoi(payload);
        break;
      }
      case 'V': {
        // Expects "V,2000,\r"
        char *header = strtok(SERIAL_RXDataBuffer, ","); // Expects: '?'
        char *payload = strtok(NULL, ",");               // Expects:<payload>
        char *footer = strtok(NULL, "\r");               // Expects: '\r'
        DEMANDED_Velocity = atoi(payload);
        break;
      }

      case 'C': {
        char *header = strtok(SERIAL_RXDataBuffer, ","); // Expects: '?'
        char *payload = strtok(NULL, ",");               // Expects:<payload>
        char *footer = strtok(NULL, "\r");               // Expects: '\r'
        DEMANDED_Current = (float)(atoi(payload)) / 100;
        break;
      }
      case 'A': {
        char *header = strtok(SERIAL_RXDataBuffer, ","); // Expects: '?'
        char *payload1 = strtok(NULL, ",");              // Expects:<payload>
        char *payload2 = strtok(NULL, ",");              // Expects:<payload>
        char *footer = strtok(NULL, "\r");               // Expects: '\r'
        DEMANDED_Current = (float)(atoi(payload1)) / 100;
        DEMANDED_Velocity = (float)(atoi(payload2));
        break;
      }
      case 'B': {
        char *header = strtok(SERIAL_RXDataBuffer, ","); // Expects: '?'
        char *payload1 = strtok(NULL, ",");              // Expects:<payload>
        char *payload2 = strtok(NULL, ",");              // Expects:<payload>
        char *payload3 = strtok(NULL, ",");              // Expects:<payload>
        char *footer = strtok(NULL, "\r");               // Expects: '\r'
        DEMANDED_Current = (float)(atoi(payload1)) / 100;
        DEMANDED_Velocity = (float)(atoi(payload2));
        DEMANDED_Position = (float)(atoi(payload3));
        break;
      }

      case 'J': {
        JOYSTICK_TISR.attach(&JOYSTICK_ISR_Read, 0.005);
        break;
      }

      case 'S': {
        JOYSTICK_TISR
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

    if (IMU_Flag) {
      SERIAL_SystemStatus_ISRR.attach(&SERIAL_SystemStatus_ISR,
                                      SYSTEMTIMEOUTINTERVAL);
      imu1.setmode(OPERATION_MODE_NDOF);
      imu2.setmode(OPERATION_MODE_NDOF);
      IMU1_Angle();
      IMU2_Angle();
      // IMU1_Acceleration();
      IMU1_Linear_Acceleration();
      Acceleration_Computation();
      FD_Computation();
      IMU_Flag = 0;
    }
    if ((Cascade_Mode == 3) &&
        ((PID_CURRENT_INITIALISED) && (PID_VELOCITY_INITIALISED) &&
         (PID_POSITION_INITIALISED))) {
      if (PID_Position_Flag) {
        PID_Position_Computation();
        PID_Position_Flag = 0;
      }
      if (PID_Velocity_Flag) {
        PID_Velocity_Computation();
        PID_Velocity_Flag = 0;
      }
      if (PID_Current_Flag) {
        PID_Current_Computation();
        PID_Current_Flag = 0;
      }

    } else if ((Cascade_Mode == 2) &&
               ((PID_CURRENT_INITIALISED) && (PID_VELOCITY_INITIALISED))) {
      if (PID_Velocity_Flag) {
        PID_Velocity_Computation();
        PID_Velocity_Flag = 0;
      }
      if (PID_Current_Flag) {
        PID_Current_Computation();
        PID_Current_Flag = 0;
      }

    } else if ((Cascade_Mode == 1) && (PID_CURRENT_INITIALISED)) {
      if (PID_Current_Flag) {
        PID_Current_Computation();
        PID_Current_Flag = 0;
      }
    }

    if (JOYSTICK_Read_Flag) {
      JOYSTICK_Read();
      JOYSTICK_Read_Flag = 0;
    }
    if (SERIAL_Print_Flag) {
      SERIAL_Print();
      SERIAL_Print_Flag = 0;
    }
    if (CURRENT_Sensor_Flag) {
      CURRENT_Sensor_Read();
      CURRENT_Sensor_Flag = 0;
    }
    if (SERIAL_SystemStatus_Flag) {
      SERIAL_SystemStatus();
      SERIAL_SystemStatus_Flag = 0;
    }
  }
}

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
  //   if (Cascade_Mode == 1) {
  //     PC.printf("%f %f %f \n\r", TIME1.read(), DEMANDED_Current,
  //     MOTOR_Current);
  //   } else if (Cascade_Mode == 2) {
  //     PC.printf("%f %f %f %f %f \n\r", TIME1.read(), DEMANDED_Current_Total,
  //               MOTOR_Current, DEMANDED_Velocity, ENCODER_RPM);
  //   } else if (Cascade_Mode == 3) {
  //     float t1 = TIME1.read();
  //     PC.printf("%f %f %f %f %f %f %f \n\r", TIME1.read(),
  //     DEMANDED_Current_Total,
  //               MOTOR_Current, DEMANDED_Velocity_Total, ENCODER_RPM,
  //               DEMANDED_Position, LEADSCREW_Position);
  //   }

  PC.printf("%f %f %f %f %f %f %f %f %f %f %f %f %f \n\r ", TIME1.read(),
            DEMANDED_Current_Total, MOTOR_Current, DEMANDED_Velocity_Total,
            ENCODER_RPM, DEMANDED_Position, LEADSCREW_Position, IMU1_Roll,
            IMU1_Pitch, PEN_Angle, Y_DDOT_Fil4, Z_DDOT_Fil4,LENGTH_Acc);
  //   PC.printf("X1:%f Y1:%f Z1:%f YDDOT:%f ZDDOT:%f LENGTH_Acc:%f \n\r",
  //   IMU1_X_Linear_Acc,IMU1_Y_Linear_Acc,IMU1_Z_Linear_Acc,Y_DDOT_Fil4,Z_DDOT_Fil4,LENGTH_Acc);

  //   PC.printf("%f %f %f %f %f %f \n\r", TIME1.read(), IMU1_Y_Linear_Acc,
  //             IMU1_Z_Linear_Acc, Y_DDOT_Fil4, Z_DDOT_Fil4, LENGTH_Acc);

  //   PC.printf("%f %5.2f %5.2f %5.2f %5.2f %5.2f \n\r", TIME1.read(),
  //   FD_Acc_u[0],
  //             FD_OutputAcc_Fil[0], FD_OutputVel[0], FD_OutputPos_Fil[0],
  //             PEN_Angle);
  //   PC.printf(" %f %f %f \n\r", TIME1.read(), DEMANDED_Velocity,
  //   ENCODER_RPM);
  // PC.printf("AnalogIn: %f
  // %f\n\r",CURRENT_Sensor_ADC_Reading,CURRENT_Offset);
  //    printf("%f_%f \n",L_PWMSpeed,R_PWMSpeed);
  // PC.printf(" RSpeed: %f, ENCODER_Count: %i Encoder Change:%f \n\r",
  // ENCODER_RPM, ENCODER_Count,ENCODER_Change);
  //    printf("ENCODER_Count: %f \n\r",ENCODER_Count);
  //   PC.printf("LSwitch State: %i \n\r", LSWITCH_Flag);
  //   PC.printf("Time: %f  Demanded Position: %f Leadscrew Position: %f \n\r",
  //             TIME1.read(), DEMANDED_Position, LEADSCREW_Position);

  //   PC.printf("Current Time: %f Demanded Position: %f Leadscrew Position:
  //   %f
  //   "
  //             "EncoderCounts:%i \n\r",
  //             TIME1.read(), DEMANDED_Position, LEADSCREW_Position,
  //             ENCODER_Count);
  //   PC.printf("ADC_Current: %f, Current:%f \n\r",
  //   CURRENT_Sensor_ADC_Reading,
  //             MOTOR_Current);
  //   PC.printf("Time: %f  Demanded Current: %f Leadscrew Current: %f
  //   MOTOR_Speed: "
  //             "%f \n\r",
  //             TIME1.read(), DEMANDED_Current, MOTOR_Current,
  //             MOTOR_Speed_PID);
  // PC.printf("D: %f, Actual: %f, PWM: %f \n\r", DEMANDED_Velocity,
  //       ENCODER_RPM, MOTOR_Speed_PID);
}

void SetSpeed(int MOTOR_Speed) {
  if (MOTOR_Speed >= 0) {
    R_EN = 1; // Enable motor to spin rightwards
    L_EN = 1; // Disable motor to spin leftwards
    //        R_PWMSpeed=map(abs(MOTOR_Speed),0,100,0,1.0);
    //        R_PWM.write(R_PWMSpeed);
    //        L_PWMSpeed=0;
    //        L_PWM.write(L_PWMSpeed);
    L_PWMSpeed = map(abs(MOTOR_Speed), 0, 100, 0, MAX_PWM);
    L_PWM.write(L_PWMSpeed);
    R_PWMSpeed = 0;
    R_PWM.write(R_PWMSpeed);
  } else {
    L_EN = 1; // Enable motor to spin leftwards
    R_EN = 1; // Disable motor to spin rightwards

    R_PWMSpeed = map(abs(MOTOR_Speed), 0, 100, 0, MAX_PWM);
    R_PWM.write(R_PWMSpeed);
    L_PWMSpeed = 0;
    L_PWM.write(L_PWMSpeed);
  }
}

void JOYSTICK_Read() {
  // float Xpos=1-XJoystick.read(); //inverts the horizontal joystick position
  JOYSTICK_Y_Position =
      1 - JOYSTICK_Y.read(); // inverts the vertical Joystick position
  MOTOR_Speed = map(JOYSTICK_Y_Position, 0.0, 1.0, -100,
                    100); // maps X position to pwm of motor
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

// encoder event for the interrupt call
void ENCODER_Event() {
  if ((RH_ENCODER_A) == 1) {
    if ((RH_ENCODER_B) == 0) {
      ENCODER_Count--;
    } else {
      ENCODER_Count++;
    }
  } else {
    if ((RH_ENCODER_B) == 0) {
      ENCODER_Count++;
    } else {
      ENCODER_Count--;
    }
  }
}

void ENCODER_Check() {
  TIME1_Current = TIME1.read();
  TIME1_Sample_Duration = TIME1_Current - TIME1_Previous;
  TIME1_Previous = TIME1_Current;

  // since encoder feedback resolution is 17 for 1 revolution (shaft
  ENCODER_Change = ENCODER_Count - ENCODER_Old_Count;
  //   ENCODER_RPM = (float)(ENCODER_Change / (ENCODER_CPR *
  //   TIME1_Sample_Duration) *
  //                         60); // right wheel RPM
  ENCODER_RPM_Raw =
      (float)(ENCODER_Change / (ENCODER_CPR * TIME1_Sample_Duration) *
              60); // right wheel RPM
  ENCODER_RPM = Moving_Average3(ENCODER_RPM_Raw);
  //    ENCODER_Speed = ENCODER_RPM * 2 * 3.1415 * 0.05; //velocity=r*w
  //    (radius of wheel is 5cm) ENCODER_Speed=60*ENCODER_RPM;
  ENCODER_Old_Count = ENCODER_Count;
  LEADSCREW_Position = (float)LEADSCREW_LEAD / ENCODER_CPR * ENCODER_Count;
}

void LSWITCH_Home() {
  while (LSWITCH_Complete_Home == 0) {
    while (LSWITCH_Flag == 0) {
      SetSpeed(35); // Lift platform to hit LSWTICH
    }
    SetSpeed(0);
    thread_sleep_for(LSWITCH_SLEEP_DURATION);
    while (LSWITCH_Flag == 1) {
      SetSpeed(-35); // Lower platform to release LSWITCH
    }
    SetSpeed(0);
    thread_sleep_for(LSWITCH_SLEEP_DURATION);
    while (LSWITCH_Flag == 0) {
      SetSpeed(30); // Lift platform to hit LSWTICH at slower speed
    }
    SetSpeed(0);
    thread_sleep_for(LSWITCH_SLEEP_DURATION);
    while (LSWITCH_Flag == 1) {
      SetSpeed(-35); // Lower platform to hit LSWTICH at slower speed
    }
    SetSpeed(-35);
    thread_sleep_for(200); // initial leadscrew distance buffer
    SetSpeed(0);
    thread_sleep_for(LSWITCH_SLEEP_DURATION);
    LEADSCREW_Position = 0; // Resets Leadscrew position
    ENCODER_Count = 0;      // Resets Encoder position
    LSWITCH_Complete_Home = 1;
  }
}

float CURRENT_Sensor_Offset() {
  thread_sleep_for(200);
  float ADC_Accumulated_Readings = 0;
  for (int counter = 0; counter < 1000; counter++) {
    ADC_Accumulated_Readings += CURRENT_Sensor.read();
  }
  float Averaged_Current_Offset = 0.5 - (ADC_Accumulated_Readings / 1000);
  return Averaged_Current_Offset;
}
void CURRENT_Sensor_Read() {
  CURRENT_Sensor_ADC_Reading = CURRENT_Sensor.read() + CURRENT_Offset;
  MOTOR_Current_Raw = map(CURRENT_Sensor_ADC_Reading, 0.0, 1.0, -27.5, 27.5);
  CURRENT_Filter_Data[1] = CURRENT_Filter_Alpha * MOTOR_Current_Raw +
                           (1 - CURRENT_Filter_Alpha) * CURRENT_Filter_Data[0];
  CURRENT_Filter_Data[0] = CURRENT_Filter_Data[1];
  MOTOR_Current = CURRENT_Filter_Data[1];
  // MOTOR_Current = map(CURRENT_Sensor_ADC_Reading, 0.0, 1.0, -16.67,16.67);
}

void Cascade_Initialisation(int mode) {
  Cascade_Mode = mode;
  if (Cascade_Mode == 1) {
    PID_Current_Initialisation();
  } else if (Cascade_Mode == 2) {
    PID_Current_Initialisation();
    PID_Velocity_Initialisation();
  } else if (Cascade_Mode == 3) {
    PID_Current_Initialisation();
    PID_Velocity_Initialisation();
    PID_Position_Initialisation();
  }
}
void PID_Position_Initialisation() {
  PID_Position_TISR.attach(&PID_Position_ISR, PID_POSITION_RATE);
  PID_Position.setInputLimits(0, LEADSCREW_MAX_RANGE);
  PID_Position.setOutputLimits(-2000, 2000);
  PID_Position.setMode(AUTO_MODE);
  PID_POSITION_INITIALISED = 1;
}
void PID_Velocity_Initialisation() {
  PID_Velocity_TISR.attach(&PID_Velocity_ISR, PID_VELOCITY_RATE);
  PID_Velocity.setInputLimits(-MAX_MOTORSPEED, MAX_MOTORSPEED);
  PID_Velocity.setOutputLimits(-20, 20);
  PID_Velocity.setMode(AUTO_MODE);
  PID_VELOCITY_INITIALISED = 1;
}

void PID_Current_Initialisation() {
  PID_Current_TISR.attach(&PID_Current_ISR, PID_CURRENT_RATE);
  PID_Current.setInputLimits(-CURRENT_MAX_RANGE, CURRENT_MAX_RANGE);
  PID_Current.setOutputLimits(-100, 100);
  PID_Current.setMode(AUTO_MODE);
  PID_CURRENT_INITIALISED = 1;
}

void PID_Position_Computation() {
  // Imposing limits to leadscrew demanded position
  if (DEMANDED_Position > LEADSCREW_MAX_RANGE) {
    DEMANDED_Position = LEADSCREW_MAX_RANGE;
  }
  if (DEMANDED_Position < 0) {
    DEMANDED_Position = 0;
  }
  PID_Position.setSetPoint(DEMANDED_Position);
  PID_Position.setProcessValue(LEADSCREW_Position);
  ERROR_Pos = -PID_Position.compute();
}

void PID_Velocity_Computation() {
  if (DEMANDED_Velocity > MAX_MOTORSPEED) {
    DEMANDED_Velocity = MAX_MOTORSPEED;
  } else if (DEMANDED_Velocity < -MAX_MOTORSPEED) {
    DEMANDED_Velocity = -MAX_MOTORSPEED;
  }
  DEMANDED_Velocity_Total = DEMANDED_Velocity - ERROR_Pos;
  PID_Velocity.setSetPoint(DEMANDED_Velocity_Total);
  PID_Velocity.setProcessValue(ENCODER_RPM);
  ERROR_Vel = -PID_Velocity.compute();
}

void PID_Current_Computation() {
  if (DEMANDED_Current > CURRENT_MAX_RANGE) {
    DEMANDED_Current = CURRENT_MAX_RANGE;
  }
  if (DEMANDED_Current < -CURRENT_MAX_RANGE) {
    DEMANDED_Current = -CURRENT_MAX_RANGE;
  }
  DEMANDED_Current_Total = DEMANDED_Current + ERROR_Vel;
  PID_Current.setSetPoint(DEMANDED_Current_Total);
  PID_Current.setProcessValue(MOTOR_Current);
  MOTOR_Speed_PID = PID_Current.compute();
  if ((abs(DEMANDED_Current_Total) > HARDWARE_CMAX) ||
      (abs(DEMANDED_Velocity_Total) > HARDWARE_VMAX) ||
      (DEMANDED_Position > HARDWARE_PMAX) ||
      (DEMANDED_Position < HARDWARE_PMIN)) {
    SetSpeed(0); // Safety Limits
    //   PC.printf("LIMIT HIT");
  } else {
    SetSpeed(MOTOR_Speed_PID);
  }
}

void SERIAL_SystemStatus() {
  SetSpeed(0); // Safety Limits
  imu1.reset();
  imu2.reset();
}

void IMU_Init() {
  imu1.reset();
  imu2.reset();
  //   thread_sleep_for(200);
  imu1.set_accel_units(MPERSPERS); // Sets Acceleration output to m/s2
  imu1.set_angle_units(DEGREES);   // Sets Angle output to degrees
  imu2.set_accel_units(MPERSPERS); // Sets Acceleration output to m/s2
  imu2.set_angle_units(DEGREES);   // Sets Angle output to degrees
  imu1.setmode(OPERATION_MODE_NDOF);
  imu2.setmode(OPERATION_MODE_NDOF);
}

void IMU1_Angle() {
  // float t1=TIME1.read();
  imu1.get_angles();
  IMU1_Pitch = imu1.euler.pitch * PI / 180;
  IMU1_Roll = imu1.euler.roll * PI / 180;
  //   PEN_Angle = sqrt((IMU1_Pitch * IMU1_Pitch) + (IMU1_Roll * IMU1_Roll));
}
void IMU2_Angle() {
  imu2.get_angles();
  IMU2_Pitch = imu2.euler.pitch * PI / 180;
  IMU2_Roll = imu2.euler.roll * PI / 180;
  PEN_Angle = sqrt((IMU2_Pitch * IMU2_Pitch) + (IMU2_Roll * IMU2_Roll));
}

void IMU1_Acceleration() {
  imu1.get_accel();
  IMU1_X_Acc = imu1.accel.x;
  IMU1_Y_Acc = imu1.accel.y;
  IMU1_Z_Acc = imu1.accel.z;
}

void IMU2_Acceleration() {
  imu2.get_accel();
  IMU2_X_Acc = imu2.accel.x;
  IMU2_Y_Acc = imu2.accel.y;
  IMU2_Z_Acc = imu2.accel.z;
}

void IMU1_Linear_Acceleration() {
  float t1 = TIME1.read();
  imu1.get_lia();
  IMU1_X_Linear_Acc = imu1.lia.x;
  IMU1_Y_Linear_Acc = imu1.lia.y;
  IMU1_Z_Linear_Acc = imu1.lia.z;
}

void IMU2_Linear_Acceleration() {
  imu2.get_lia();
  IMU2_X_Linear_Acc = imu2.lia.x;
  IMU2_Y_Linear_Acc = imu2.lia.y;
  IMU2_Z_Linear_Acc = imu2.lia.z;
}

void Acceleration_Computation() {
  IMU1_Y_Filtered = Moving_Average4_1(IMU1_Y_Linear_Acc);
  IMU1_X_Filtered = Moving_Average4_2(IMU1_X_Linear_Acc);
  IMU1_Z_Filtered = Moving_Average2(IMU1_Z_Linear_Acc);
  Z_DDOT_Fil4 =
      IMU1_Y_Filtered * sin(IMU1_Pitch) - IMU1_X_Filtered * sin(IMU1_Roll) +
      IMU1_Z_Filtered *
          cos(sqrt((IMU1_Pitch * IMU1_Pitch) + (IMU1_Roll * IMU1_Roll)));
  //   Z_DDOT = IMU1_Y_Linear_Acc * sin(IMU1_Pitch) -
  //            IMU1_X_Linear_Acc * sin(IMU1_Roll) +
  //            IMU1_Z_Linear_Acc *
  //                cos(sqrt((IMU1_Pitch * IMU1_Pitch) + (IMU1_Roll *
  //                IMU1_Roll)));
  y_ddot =
      IMU1_Y_Filtered * cos(IMU1_Pitch) - IMU1_Z_Filtered * sin(IMU1_Pitch);

  x_ddot = IMU1_X_Filtered * cos(IMU1_Roll) + IMU1_Z_Filtered * sin(IMU1_Roll);

  Y_DDOT_Fil4 = sqrt((x_ddot * x_ddot) + (y_ddot * y_ddot));
  // Z_DDOT_Fil5 = Moving_Average5(Z_DDOT);
  //   Z_DDOT_Fil4 = Moving_Average4_1(Z_DDOT); //THIS IS THE ONE
  //   Z_DDOT_Fil3 = Moving_Average3(Z_DDOT);
  //   Z_DDOT_Fil2 = Moving_Average2(Z_DDOT);
  //   Y_DDOT_Fil4 = Moving_Average4_2(Y_DDOT); //THIS IS THE ONE
  LENGTH_Acc = -Y_DDOT_Fil4 * sin(PEN_Angle) + Z_DDOT_Fil4 * cos(PEN_Angle);
  // PC.printf("%f %f %f %f %f %f \n\r", TIME1.read(), IMU1_X_Linear_Acc,
  //           IMU1_Y_Linear_Acc, IMU1_X_Filtered, IMU1_Y_Filtered,
  //           IMU1_Z_Filtered);
  // PC.printf("%f %f %f %f %f %f \n\r", TIME1.read(), IMU1_Pitch,
  //           IMU1_Roll, IMU1_X_Filtered, IMU1_Y_Filtered,
  //           IMU1_Z_Filtered);
}
void FD_Computation() {

  FD_Acc_u[0] = LENGTH_Acc; // Updates new acceleration result
  FD_Acc_y[0] = 0.9038 * FD_Acc_u[0] - 0.2216 * FD_Acc_u[1] -
                0.9049 * FD_Acc_u[2] + 0.2205 * FD_Acc_u[3] +
                0.03077 * FD_Acc_y[1] + 0.8471 * FD_Acc_y[2] -
                0.1837 * FD_Acc_y[3];
  FD_OutputAcc[0] = FD_Acc_y[0]; // Stores the result of Washout Filter
  FD_Acc_u[3] = FD_Acc_u[2];     // Shifts the acceleration results
  FD_Acc_u[2] = FD_Acc_u[1];
  FD_Acc_u[1] = FD_Acc_u[0];
  FD_Acc_y[3] = FD_Acc_y[2];
  FD_Acc_y[2] = FD_Acc_y[1];
  FD_Acc_y[1] = FD_Acc_y[0];

  FD_OutputAcc_Fil[0] = A_filter_alpha * (FD_OutputAcc_Fil[1] +
                                          FD_OutputAcc[0] - FD_OutputAcc[1]);

  FD_OutputVel[0] =
      IMU_INTERVAL / 2 * (FD_OutputAcc_Fil[0] + FD_OutputAcc_Fil[1]) +
      FD_OutputVel[1];

  FD_OutputPos[0] =
      IMU_INTERVAL / 2 * (FD_OutputVel[0] + FD_OutputVel[1]) + FD_OutputPos[1];

  //   FD_OutputVel_Unfiltered[0] =
  //       IMU_INTERVAL / 2 * (FD_OutputAcc[0] + FD_OutputAcc[1]) +
  //       FD_OutputVel_Unfiltered[1];
  //   FD_OutputVel_Fil[0] =
  //       V_filter_alpha *
  //       (FD_OutputVel_Fil[1] + FD_OutputVel[0] - FD_OutputVel[1]);
  //   FD_OutputPos_Unfiltered[0] =
  //       IMU_INTERVAL / 2 *
  //           (FD_OutputVel_Unfiltered[0] + FD_OutputVel_Unfiltered[1]) +
  //       FD_OutputPos_Unfiltered[1];
  //   FD_OutputPos[0] =
  //       IMU_INTERVAL / 2 * (FD_OutputVel_Fil[0] + FD_OutputVel_Fil[1]) +
  //       FD_OutputPos[1];
  FD_OutputPos_Fil[0] = P_filter_alpha * (FD_OutputPos_Fil[1] +
                                          FD_OutputPos[0] - FD_OutputPos[1]);

  DEMANDED_Position = LEADSCREW_INITIAL_POS - FD_OutputPos_Fil[0] * 1000;
  DEMANDED_Velocity = -FD_OutputVel[0] * 60 * 1000 / 8;
  float Jtot = 0.00002756; // Leadscrew Moment of Inertia
  float Kt = 0.0267;       // Motor torque constant
  DEMANDED_Current =
      ((3.1416 * Jtot) / (Kt * LEADSCREW_LEAD / 1000)) * FD_OutputAcc_Fil[0];

  FD_OutputAcc[1] = FD_OutputAcc[0];
  FD_OutputVel[1] = FD_OutputVel[0]; // Shifts the velocity results
  FD_OutputPos[1] = FD_OutputPos[0]; // Shifts the position results
  FD_OutputAcc_Fil[1] = FD_OutputAcc_Fil[0];
  //   FD_OutputVel_Fil[1] = FD_OutputVel_Fil[0];
  FD_OutputPos_Fil[1] = FD_OutputPos_Fil[0];
  //   FD_OutputVel_Unfiltered[1] = FD_OutputVel_Unfiltered[0];
  //   FD_OutputPos_Unfiltered[1] = FD_OutputPos_Unfiltered[0];
  //   PC.printf("%f %f
  //   %f\n\r",DEMANDED_Current,DEMANDED_Current_Total,FD_OutputAcc_Fil[0]);
}

float Moving_Average5(float data) {
  MA5_Data[0] =
      (data + MA5_Data[1] + MA5_Data[2] + MA5_Data[3] + MA5_Data[4]) / 5;
  MA5_Data[4] = MA5_Data[3];
  MA5_Data[3] = MA5_Data[2];
  MA5_Data[2] = MA5_Data[1];
  MA5_Data[1] = MA5_Data[0];
  return MA5_Data[0];
}
float Moving_Average4_1(float data) {
  MA4_Data1[0] = (data + MA4_Data1[1] + MA4_Data1[2] + MA4_Data1[3]) / 4;
  MA4_Data1[3] = MA4_Data1[2];
  MA4_Data1[2] = MA4_Data1[1];
  MA4_Data1[1] = MA4_Data1[0];
  return MA4_Data1[0];
}
float Moving_Average4_2(float data) {
  MA4_Data2[0] = (data + MA4_Data2[1] + MA4_Data2[2] + MA4_Data2[3]) / 4;
  MA4_Data2[3] = MA4_Data2[2];
  MA4_Data2[2] = MA4_Data2[1];
  MA4_Data2[1] = MA4_Data2[0];
  return MA4_Data2[0];
}

float Moving_Average4_3(float data) {
  MA4_Data3[0] = (data + MA4_Data3[1] + MA4_Data3[2] + MA4_Data3[3]) / 4;
  MA4_Data3[3] = MA4_Data3[2];
  MA4_Data3[2] = MA4_Data3[1];
  MA4_Data3[1] = MA4_Data3[0];
  return MA4_Data3[0];
}

float Moving_Average3(float data) {
  MA3_Data[0] = (data + MA3_Data[1] + MA3_Data[2]) / 3;
  MA3_Data[2] = MA3_Data[1];
  MA3_Data[1] = MA3_Data[0];
  return MA3_Data[0];
}
float Moving_Average2(float data) {
  MA2_Data[0] = (data + MA2_Data[1]) / 2;
  MA2_Data[1] = MA2_Data[0];
  return MA2_Data[0];
}

// ISR Functions
void IMU_ISR() { IMU_Flag = 1; }
void JOYSTICK_ISR_Read() { JOYSTICK_Read_Flag = 1; }
void MOTOR_ISR_Write() { MOTOR_Write_Flag = 1; }
void LSWITCH_Rise_ISR() { LSWITCH_Flag = 0; } // LSWITCH is released
void LSWITCH_Fall_ISR() {
  LSWITCH_Flag = 1;
  if (LSWITCH_Complete_Home) {
    MOTOR_Speed = 0; // Hardware failsafe, stops motor immediately if it
                     // crashes into LSWITCH
  }
} // LSWITCH is being pressed
void SERIAL_Print_ISR() { SERIAL_Print_Flag = 1; }
void CURRENT_SENSOR_ISR_Read() { CURRENT_Sensor_Flag = 1; }
void PID_Position_ISR() { PID_Position_Flag = 1; }
void PID_Velocity_ISR() { PID_Velocity_Flag = 1; }
void PID_Current_ISR() { PID_Current_Flag = 1; }
void SERIAL_SystemStatus_ISR() { SERIAL_SystemStatus_Flag = 1; }