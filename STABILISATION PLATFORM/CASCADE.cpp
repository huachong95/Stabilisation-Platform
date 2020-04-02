#define L_EN_PIN D10
#define R_EN_PIN D8
#define L_PWM_PIN D6
#define R_PWM_PIN D9
#define JOYSTICK_PIN A5
#define L_SWITCH_PIN D12
#define RH_ENCODER_A_PIN D3        // right motor encoder A interrupt pin
#define RH_ENCODER_B_PIN D2        // right motor encoder B interrupt pin
#define CURRENT_SENSOR_PIN A0      // ACS712 Current Sensor pin
#define ENCODER_INTERVAL 0.05      // Encoder read interval
#define LSWITCH_SLEEP_DURATION 600 // Minimum cycle switch duration required
#define LEADSCREW_LEAD 8           // Lead in mm
#define LEADSCREW_MAX_RANGE 330
#define MAX_MOTORSPEED 4500
#define ENCODER_CPR 30 // Encoder Pulses per revolution
#define MOTOR_WRITE_RATE 0.01     // Write Rate of Motor
#define PID_POSITION_RATE 0.01    // 100Hz Sample Rate of PID_Position
#define PID_VELOCITY_RATE 0.001   // 1000HzSample Rate of PID_Velocity
#define PID_CURRENT_RATE 0.0001   // 10000HzSample Rate of PID_Current
#define CURRENT_MAX_RANGE 20      // Max Amps supported by Current Sensor
#define LEADSCREW_INITIAL_POS 210 // Leadscrew initial position
#define CASCADE_MODE 3            // 1==C, 2==C&V 3==C&V&P

#include "PID.h"
#include "mbed.h"
#include <cstdio>
#include <iostream>

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
PID PID_Position(9, 10.0, 0.0, PID_POSITION_RATE);
// PID PID_Position(8, 1000.0, 0.0, PID_POSITION_RATE);
PID PID_Velocity(1.0, 20.0, 0.0, PID_VELOCITY_RATE);
PID PID_Current(85, 30.0, 0, PID_CURRENT_RATE);

Ticker MOTOR_TISR;
Ticker JOYSTICK_TISR;      // Ticker interrupt for updating of joystick position
Ticker ENCODER_Check_TISR; // Ticker interrupt for Encoder ISR
Ticker CURRENT_Sensor_TISR;
Ticker PID_Position_TISR;
Ticker PID_Velocity_TISR;
Ticker PID_Current_TISR;
Timer TIME1;

Thread CASCADE_thread(osPriorityNormal);  //Creates MOTOR thread

// VARIABLE INSTANTIATION

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
float DEMANDED_Velocity = 0.0;
float DEMANDED_Velocity_Total = 0.0;
float DEMANDED_Current = 0.0;
float DEMANDED_Current_Total = 0.0;
float MOTOR_Speed_PID = 0.0;

// FUNCTION DECLARATIONS
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

void CASCADE_Thread() {
  MOTOR_TISR.attach(&MOTOR_ISR_Write, MOTOR_WRITE_RATE);
  ENCODER_Check_TISR.attach(&ENCODER_Check, ENCODER_INTERVAL);
  L_PWM.period(0.00008);
  R_PWM.period(0.00008);
  LSWITCH.rise(&LSWITCH_Rise_ISR);
  LSWITCH.fall(&LSWITCH_Fall_ISR);
  TIME1.start(); // Startsthe TIME1 timer
  LSWITCH_Home();
  RH_ENCODER_A.rise(&ENCODER_Event);
  RH_ENCODER_A.fall(&ENCODER_Event);

  CURRENT_Sensor_TISR.attach(&CURRENT_SENSOR_ISR_Read, PID_CURRENT_RATE);
  CURRENT_Offset = CURRENT_Sensor_Offset(); // obtains the zero-offset current
                                            //   PID_Position_Initialisation();
  Cascade_Initialisation(CASCADE_MODE);
  PID_Position_Initialisation();

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
    if (CURRENT_Sensor_Flag) {
      CURRENT_Sensor_Read();
      CURRENT_Sensor_Flag = 0;
    }
  }
}

void CASCADE_Init(){
    CASCADE_thread.start(CASCADE_Thread);
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
  ENCODER_RPM = (float)(ENCODER_Change / (ENCODER_CPR * TIME1_Sample_Duration) *
                        60); // right wheel RPM
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
  CURRENT_Sensor_ADC_Reading = CURRENT_Sensor.read() - CURRENT_Offset;
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
  SetSpeed(MOTOR_Speed_PID);
}

// ISR Functions
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
void CURRENT_SENSOR_ISR_Read() { CURRENT_Sensor_Flag = 1; }
void PID_Position_ISR() { PID_Position_Flag = 1; }
void PID_Velocity_ISR() { PID_Velocity_Flag = 1; }
void PID_Current_ISR() { PID_Current_Flag = 1; }

// Inter-Thread Communication
int CASCADE_Get_CASCADE_Mode(){
    return CASCADE_MODE;
}

float CASCADE_Get_LEADSCREW_Pos(){
    return LEADSCREW_Position;
}
float CASCADE_Get_ENCODER_RPM(){
    return ENCODER_RPM;
}

float CASCADE_Get_MOTOR_Current(){
    return MOTOR_Current;
}

float CASCADE_Get_DEMANDED_Current_Total(){
    return DEMANDED_Current_Total;
}
float CASCADE_Get_DEMANDED_Current(){
    return DEMANDED_Current;
}

float CASCADE_Get_DEMANDED_Velocity_Total(){
    return DEMANDED_Velocity_Total;
}

float CASCADE_Get_DEMANDED_Velocity(){
    return DEMANDED_Velocity;
}
float CASCADE_Get_DEMANDED_Position(){
    return DEMANDED_Position;
}


