//MOTOR definitions
#define MOTORSTARTUPSPEED 0
#define MAXMOTORSPEED 120
#define READBATTERYINTERVAL 1
#define READSTATEINTERVAL 0.1
#define SETSPEEDINTERVAL 0.1  //Ticker interrupt interval for setting of motor speed
#define TESTWAITINTERVAL 1
#define CANID_LeftMotor 1   //CAN ID of left motor set as 1. Setting needs to configured in the ZLTECH motor driver application
#define CANID_RightMotor 2  //CAN ID of right motor set as 2 Setting needs to configured in the ZLTECH motor driver application
#define READY 1
#define IDLE 0

//MOTOR-SONAR definitions
#define MAXFRONTCRITICALDISTANCE 45
#define MINFRONTCRITICALDISTANCE 30
#define SIDECRITICALDISTANCE 20
#define CRITICALSPEED 0

#include "mbed.h"
#include "serial.h"
#include "led.h"
#include "sonar.h"
#include <iostream>
#include <stdio.h>

CAN MOTOR_CANBus(PA_11, PA_12,1000000);               // Creates a CAN object for CAN communication
CANMessage msg;                 // Create empty CAN message
Ticker  MOTOR_Ticker_ISR;        // Creates a ticker timer interrupt for sending motor commands
Ticker MOTOR_Monitor_ISR;        // Creates a ticker timer interrupt for reading bus voltage
Ticker MOTOR_MovementState_ISR;  //Creates a ticker timer interrupt for reading movement state of ELVIS
Thread MOTOR_thread(osPriorityNormal);  //Creates MOTOR thread

char DataL[8];  //motor data buffer for left motor
char DataR[8];  //motor data buffer for right motor
int LSpeed= MOTORSTARTUPSPEED;  //Left motor initialisation speed
int RSpeed= MOTORSTARTUPSPEED;  //Right motor initialisation speed
int SONAR_MaxMotorSpeed = MAXMOTORSPEED;    //Max allowable speed allowed after checking SONAR sensors
bool MOTOR_COMMAND_STATE=IDLE; //initialises motor command state as low
bool MOTOR_CAN_STATE=IDLE; //initialises motor CAN_read state as low
char MOVEMENT_STATE='S';    //initialises robot movement state as 'STOP' since startup speed is stationary
int FRONTCRITICALDISTANCE= MAXFRONTCRITICALDISTANCE;
signed long EncoderLPosition;   //Variable for left motor encoder position readback
signed long EncoderRPosition;   //Variable for right motor encoder position readback
float EncoderLCurrent;          //Variable for left motor current readback
float EncoderRCurrent;          //Variable for right motor current readback
signed int EncoderLSpeed=0;     //Variable for left motor encoder speed readback
signed int EncoderRSpeed=0;     //Variable for right motor encoder speed readback
int BatteryVoltage =42;         //Variable for battery voltage. System initialises at full battery voltage
int SONAR1DISTANCE = 90000;     //Variable for storing object distance from SONAR1
int SONAR2DISTANCE = 90000;     //Variable for storing object distance from SONAR2
int SONAR3DISTANCE = 90000;     //Variable for storing object distance from SONAR3
int SONAR4DISTANCE = 90000;     //Variable for storing object distance from SONAR4

//Function declarations
void MOTOR_SetState();
void MOTOR_WriteLSpeed(int leftrpmSpeed);
void MOTOR_WriteRSpeed(int righttrpmSpeed);
void MOTOR_CAN_Read();
void MOTOR_Read();
void MOTOR_CheckDistance();
void MOTOR_MovementState();
void MOTOR_Monitor();

void MOTOR_Thread()
{
    while(1) {
        SONAR1DISTANCE = SONAR_GetSONAR1();
        SONAR2DISTANCE = SONAR_GetSONAR2();
        SONAR3DISTANCE = SONAR_GetSONAR3();
        SONAR4DISTANCE = SONAR_GetSONAR4();
        if ((SONAR2DISTANCE<MAXFRONTCRITICALDISTANCE||SONAR3DISTANCE<MAXFRONTCRITICALDISTANCE)||(SONAR1DISTANCE<SIDECRITICALDISTANCE||SONAR4DISTANCE<SIDECRITICALDISTANCE)) {
            MOTOR_CheckDistance();
//            wait(0.002); //Delay of 1 millisecond required for system to run
        } else {
            FRONTCRITICALDISTANCE = MAXFRONTCRITICALDISTANCE;
            SONAR_MaxMotorSpeed=MAXMOTORSPEED;
        }
        switch (MOTOR_COMMAND_STATE) {
            case IDLE:
                break;
            case READY:
                if ((SONAR2DISTANCE<MAXFRONTCRITICALDISTANCE||SONAR3DISTANCE<MAXFRONTCRITICALDISTANCE)||(SONAR1DISTANCE<SIDECRITICALDISTANCE||SONAR4DISTANCE<SIDECRITICALDISTANCE)) {
                    MOTOR_CheckDistance();
                } else {
                    SONAR_MaxMotorSpeed=MAXMOTORSPEED;
                    FRONTCRITICALDISTANCE=MAXFRONTCRITICALDISTANCE;
                }
                MOTOR_WriteLSpeed(LSpeed);  //Writes left speed to left motor
                MOTOR_WriteRSpeed(RSpeed);  //Writes right speed to right motor
                MOTOR_COMMAND_STATE=IDLE; //resets the motor into the idle state
                break;
        }
    }
}

void MOTOR_SetSpeed(int Data1,int Data2)
{
    LSpeed = Data1;     //Reads left speed obtained from SERIAL thread
    RSpeed = Data2;     //Reads righy speed obtained from SERIAL thread
}
int MOTOR_GetLSpeed()
{
    return EncoderLSpeed;   //Returns left encoder speed if requested by other threads
}
int MOTOR_GetRSpeed()
{
    return EncoderRSpeed;   //Returns right encoder speed if requested by other threads
}
long int MOTOR_GetLPosition()
{
    return EncoderLPosition;    //Returns left encoder position if requested by other threads
}
long int MOTOR_GetRPosition()
{
    return EncoderRPosition;    //Returns right encoder position if requested by other threads
}


void MOTOR_Init()
{
    /*
    For full motor driver initialisation steps and packet format, refer to ZLTECH motor controller documentation
    Step 1: Set motor driver modes (Speed, Position or Torque mode). SPEED mode is used for ELVIS
    Step 2: Set acceleration and deceleration limits for motors
    Step 3: Set the initial startup speed of motors
    Step 4: Enable motors
    */
    //Sets motor drivers to speed mode
    DataL[0] = DataR[0] = (char)(0x00);
    DataL[1] = DataR[1] = (char)(0xDA);     //0xDA, NOT 0xFA (as indicated in controller documentation) must be used if data packet is to be filtered and set to different CAN IDs
    DataL[2] = DataR[2] = (char)(0x00);
    DataL[3] = DataR[3] = (char)(0x19);
    DataL[4] = DataR[4] = (char)(0x00);
    DataL[5] = DataR[5] = (char)(0x00);
    DataL[6] = DataR[6] = (char)(0x00);
    DataL[7] = DataR[7] = (char)(0x2F);
    if(MOTOR_CANBus.write(CANMessage(CANID_LeftMotor,DataL,8))) {   //writes the message packet DataL to CANID of left motor
//        printf("Left Motor set to speed mode \n");
    }
    if(MOTOR_CANBus.write(CANMessage(CANID_RightMotor,DataR,8))) {  //writes the message packet DataR to CANID of right motor
//        printf("Right Motor set to speed mode \n");
    }
    wait(0.1);
//sets the acceleration and deceleration time of motors
    DataL[0] = DataR[0] = (char)(0x00);
    DataL[1] = DataR[1] = (char)(0xDA);
    DataL[2] = DataR[2] = (char)(0x00);
    DataL[3] = DataR[3] = (char)(0x13);
    DataL[4] = DataR[4] = (char)(0x00);
    DataL[5] = DataR[5] = (char)(0x00);
//  DataL[6] = DataR[6] = (char)(0x0A); //Acceleration profile of 10 * 100ms
//  DataL[7] = DataR[7] = (char)(0x0B); //Acceleration profile of 10 * 100ms
    DataL[6] = DataR[6] = (char)(0xAA); //Acceleration profile of 170 * 100ms. Through iterative testing, this longer acceleration and deceleration profiles significantly improved joystick usability and reduced skidding
    DataL[7] = DataR[7] = (char)(0xAA); //Acceleration profile of 170 * 100ms
    if(MOTOR_CANBus.write(CANMessage(CANID_LeftMotor,DataL,8))) {
//        printf("Sets the acceleration and deceleration time for left motor \n");
    }
    if(MOTOR_CANBus.write(CANMessage(CANID_RightMotor,DataR,8))) {
//        printf("Sets the acceleration and deceleration time for right motor \n");
    }
    wait(0.1);
//Sets the inital speed of motors
    DataL[0] =  DataR[0] = (char)(0x00);
    DataL[1] =  DataR[1] = (char)(0xDA);
    DataL[2] =  DataR[2] = (char)(0x00);
    DataL[3] =  DataR[3] = (char)(0x11);
    DataL[4] =  DataR[4] = (char)(0x00);
    DataL[5] =  DataR[5] = (char)(0x00);
    DataL[6] =  DataR[6] = (char)(0x00);
    DataL[7] =  DataR[7] = (char)(0x00);
    if(MOTOR_CANBus.write(CANMessage(CANID_LeftMotor,DataL,8))) {
//        printf("Sets initial speed of 0 for left motor \n");
    }
    if(MOTOR_CANBus.write(CANMessage(CANID_RightMotor,DataR,8))) {
//        printf("Sets initial speed of 0 for right motor \n");
    }
    wait(0.1);
//Enables the motors
    DataL[0] = DataR[0] = (char)(0x00);
    DataL[1] = DataR[1] = (char)(0xDA);
    DataL[2] = DataR[2] = (char)(0x00);
    DataL[3] = DataR[3] = (char)(0x10);
    DataL[4] = DataR[4] = (char)(0x00);
    DataL[5] = DataR[5] = (char)(0x00);
    DataL[6] = DataR[6] = (char)(0x00);
    DataL[7] = DataR[7] = (char)(0x1F);
    if(MOTOR_CANBus.write(CANMessage(CANID_LeftMotor,DataL,8))) {
//        printf("Enables the left motor \n");
    }
    if(MOTOR_CANBus.write(CANMessage(CANID_RightMotor,DataR,8))) {
//        printf("Enables the right motor \n");
    }
    wait(0.1);
    MOTOR_Ticker_ISR.attach(&MOTOR_SetState,SETSPEEDINTERVAL); //Attaches ticker timer interrupt which send motor commands at specified interval
    MOTOR_Monitor_ISR.attach(&MOTOR_Monitor,READBATTERYINTERVAL);
    MOTOR_MovementState_ISR.attach(&MOTOR_MovementState,READSTATEINTERVAL); //Attaches ticker timr interrupt which interprets motor states from encoders at specified intervals
    MOTOR_CANBus.attach(&MOTOR_CAN_Read,CAN::RxIrq);         //Attaches CAN Bus RX interrupt, activating CAN Read ISR upon receiving any CAN information
    MOTOR_thread.start(MOTOR_Thread);       //Creates the MOTOR thread
}

void MOTOR_WriteLSpeed(int leftrpmSpeed)
{
    if (leftrpmSpeed>=0) {
        if (leftrpmSpeed>SONAR_MaxMotorSpeed) {
            leftrpmSpeed=SONAR_MaxMotorSpeed;   //Implements maximum left motor speed limit based on SONAR MAX SPEED
        }
        uint16_t setLValue=leftrpmSpeed*8192/3000;  //Scaling factor obtained by ZLTECH motor controller documentation. DataL[6] and DataL[7] determines the motor speed
        DataL[0] = (char)(0x00);
        DataL[1] = (char)(0xDA);
        DataL[2] = (char)(0x00);
        DataL[3] = (char)(0x11);
        DataL[4] = (char)(0x00);
        DataL[5] = (char)(0x00);
        DataL[6] = setLValue>>8; //high byte
        DataL[7] = setLValue & 0x00FF; //low byte
        MOTOR_CANBus.write(CANMessage(CANID_LeftMotor,DataL,8)); //Writes motor speed packet to CAN ID of left motor
    } else if (leftrpmSpeed<0) {
        if (abs(leftrpmSpeed)>MAXMOTORSPEED) {
            leftrpmSpeed=-MAXMOTORSPEED;    //Implements minimum left motor speed limit
        }
        uint16_t setLValue=65535+(leftrpmSpeed*8192/3000);  //for motor to spin in reverse, speed is set to FFFF(65535)+ (negative speed)
        DataL[0] = (char)(0x00);
        DataL[1] = (char)(0xDA);
        DataL[2] = (char)(0x00);
        DataL[3] = (char)(0x11);
        DataL[4] = (char)(0xFF);
        DataL[5] = (char)(0xFF);
        DataL[6] = setLValue>>8; //high byte
        DataL[7] = setLValue & 0x00FF; //low byte
        MOTOR_CANBus.write(CANMessage(CANID_LeftMotor,DataL,8)); //Writes motor speed packet to CAN ID of left motor
    }
}

void MOTOR_WriteRSpeed(int rightrpmSpeed)
{
    if (rightrpmSpeed>0) {
        if (rightrpmSpeed>SONAR_MaxMotorSpeed) {
            rightrpmSpeed=SONAR_MaxMotorSpeed;  //Implements maximum right motor speed limit based on SONAR MAX SPEED
        }
        uint16_t setRValue=65535-(rightrpmSpeed*8192/3000); //since motor wheel is flipped in opposite orientation on differential drive robot, negative speed values are set for right motor to allow robot to move forward
        DataR[0] = (char)(0x00);
        DataR[1] = (char)(0xDA);
        DataR[2] = (char)(0x00);
        DataR[3] = (char)(0x11);
        DataR[4] = (char)(0xFF);
        DataR[5] = (char)(0xFF);
        DataR[6] = setRValue>>8; //high byte
        DataR[7] = setRValue & 0x00FF; //low byte
        MOTOR_CANBus.write(CANMessage(CANID_RightMotor,DataR,8));   //Writes motor speed packet to CAN ID of right moor
    }

    else if (rightrpmSpeed<=0) {
        if (abs(rightrpmSpeed)>MAXMOTORSPEED) {
            rightrpmSpeed=-MAXMOTORSPEED;   //implements minimum right motor speed limit
        }
        uint16_t setRValue=abs(rightrpmSpeed)*8192/3000;
        DataR[0] = (char)(0x00);
        DataR[1] = (char)(0xDA);
        DataR[2] = (char)(0x00);
        DataR[3] = (char)(0x11);
        DataR[4] = (char)(0x00);
        DataR[5] = (char)(0x00);
        DataR[6] = setRValue>>8; //high byte
        DataR[7] = setRValue & 0x00FF; //low byte
        MOTOR_CANBus.write(CANMessage(CANID_RightMotor,DataR,8));   //Writes motor speed packet to CAN ID of right motor
    }
}

void MOTOR_SetState()   //Ticker Interrupt ISR to set motor speed commands
{
    MOTOR_COMMAND_STATE=READY;
}

void MOTOR_CAN_Read()   //CAN RX Interrupt ISR to read CAN heartbeat messages (encoder speed,position, current feedback)
{
    if (MOTOR_CANBus.read(msg)) {   //needs to read the CANBUS msg before MOTOR_Read function can be implemented
//    printf("ID: %i ,%02X %02X %02X %02X %02X %02X %02X %02X \n\r",msg.id, msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
        MOTOR_Read();
    }
}

void MOTOR_Read()
{
    /*
    As CAN messages replies are returned from motor drivers for every command, CAN message filtering must be done to filter out only relevant CAN heartbeat messages
    The type of frequency of CAN Heartbeat messages can be configured in the ZLTECH motor application
    */
    if((msg.id==1)&&(msg.data[1]==0xFE)) {  //Filters out CAN message of left motor and Heartbeat message (2nd byte of a heartbeat message is 0xFE)
        if(msg.data[3]==0x20) { //Filters out position message bytes in 1st line of heartbeat messages (4th byte is 0x20)
            EncoderLPosition= (signed long)((((((msg.data[4]<<8)+msg.data[5])<<8)+msg.data[6])<<8)+msg.data[7]);
        } else if(msg.data[3]==0x21) { //Filters out current and speed message bytes in 2nd line of heartbeat messages (4th byte is 0x21)
            EncoderLSpeed=((signed short)((msg.data[6]<<8)+msg.data[7]))*3000/8192;
        }
    }
    if((msg.id==2)&&(msg.data[1]==0xFE)) { //Filters out CAN message of right motor and Heartbeat message (2nd byte of a heartbeat message is 0xFE)
        if(msg.data[3]==0x20) { //Filters out position message bytes in 1st line of heartbeat messages (4th byte is 0x20)
            EncoderRPosition= (signed long)-((((((msg.data[4]<<8)+msg.data[5])<<8)+msg.data[6])<<8)+msg.data[7]);
        } else if(msg.data[3]==0x21) { //Filters out current and speed message bytes in 2nd line of heartbeat messages (4th byte is 0x21)
            EncoderRSpeed=((signed short)-((msg.data[6]<<8)+msg.data[7]))*3000/8192;
        }
    }
    if ((msg.id==2)&&(msg.data[1]==0xDB)&&(msg.data[3]==0xE1)) {
        BatteryVoltage=msg.data[7];
    }
}

void MOTOR_MovementState()  //checks ELVIS movement state
{
    if ((EncoderLSpeed>0)&&(EncoderRSpeed>0)) { //Both motors spinning forward
        MOVEMENT_STATE='F';
    } else if ((EncoderLSpeed>=0)&&(EncoderRSpeed<0)) { //Left motor spinning forward, while right motor spinning backwards (SPIN right)
        MOVEMENT_STATE='R';
    } else if ((EncoderLSpeed<0)&&(EncoderRSpeed>=0)) { //Left motor spinning backwards, right motor spinning forward (SPIN left)
        MOVEMENT_STATE='L';
    } else if ((EncoderLSpeed<0)&&(EncoderRSpeed<0)) {  //Both motors spinning backwards
        MOVEMENT_STATE='B';
    } else {
        MOVEMENT_STATE='S'; //Motor in STOP state
    }
}

void MOTOR_CheckDistance()  //MOTOR-SONAR obstacle detection algorithm
{
    MOTOR_MovementState();  //Checks the Movement State of ELVIS
//    FRONTCRITICALDISTANCE = (((MAXFRONTCRITICALDISTANCE-MINFRONTCRITICALDISTANCE)/(MAXMOTORSPEED))*((EncoderLSpeed+EncoderRSpeed)/2))+MINFRONTCRITICALDISTANCE;
    FRONTCRITICALDISTANCE =(MAXFRONTCRITICALDISTANCE-MINFRONTCRITICALDISTANCE)*(((EncoderLSpeed+EncoderRSpeed)/2))/120+MINFRONTCRITICALDISTANCE;
    printf("FrontCriticalDistance = %i \n\r",FRONTCRITICALDISTANCE);
//FRONTCRITICALDISTANCE = 70;
//if (((EncoderLSpeed+EncoderRSpeed)/2)>70){
//    FRONTCRITICALDISTANCE = 50;
//    }
//    else if (((EncoderLSpeed+EncoderRSpeed)/2)<=70){
//        FRONTCRITICALDISTANCE=5;
//        }

    switch (MOVEMENT_STATE) {
        /*
        Sonar sensor inputs depend on movement state of robot
        FORWARD State: F = checks front SONARs
        BACKWARD State: B = checks no SONARs
        LEFT Spin State: L = checks left SONAR
        RIGHT Spin State: R = checks right SONAR
        */
        case 'F':
            if (SONAR2DISTANCE<FRONTCRITICALDISTANCE||SONAR3DISTANCE<FRONTCRITICALDISTANCE) { //Either front SONARs detect an obstacle
                SONAR_MaxMotorSpeed=CRITICALSPEED;  //SONAR MAX Speed only applied in MOTOR_WriteLSpeed and MOTOR_WriteRSpeed for forward motion, this allows ELVIS to reverse out of obstacles and minimum motor speed not limited
                MOTOR_WriteLSpeed(LSpeed);
                MOTOR_WriteRSpeed(RSpeed);
            } else {
                SONAR_MaxMotorSpeed=MAXMOTORSPEED;
            }
            break;

        case 'B':
            SONAR_MaxMotorSpeed=MAXMOTORSPEED;
            break;

        case 'L':
            if (SONAR1DISTANCE<SIDECRITICALDISTANCE) {   //Obstacles detected on left of ELVIS
                if (LSpeed<0&&RSpeed>0) {   //Only if ELVIS is continuing to spin leftwards, ELVIS will not be allowed to move
                    LSpeed=0;
                    RSpeed=0;
                    MOTOR_WriteLSpeed(LSpeed);
                    MOTOR_WriteRSpeed(RSpeed);
                }
            } else {
                SONAR_MaxMotorSpeed=MAXMOTORSPEED;
            }
            break;

        case 'R':
            if (SONAR4DISTANCE<SIDECRITICALDISTANCE) {   //Obstacles detected on right of ELVIS
                if (RSpeed<0&&LSpeed>0) {   //Only if ELVIS is continuing to spin rightwards, ELVIS will not be allowed to move
                    LSpeed=0;
                    RSpeed=0;
                    MOTOR_WriteLSpeed(LSpeed);
                    MOTOR_WriteRSpeed(RSpeed);
                }
            } else {
                SONAR_MaxMotorSpeed=MAXMOTORSPEED;
            }
            break;

        case 'S':
            if (SONAR1DISTANCE<SIDECRITICALDISTANCE) { //Obstacles detected on left of ELVIS
                if (LSpeed<0&&RSpeed>0) {   //If ELVIS is about to spin leftwards
                    LSpeed=0;
                    RSpeed=0;
                    MOTOR_WriteLSpeed(LSpeed);
                    MOTOR_WriteRSpeed(RSpeed);

                }
            }
            if (SONAR4DISTANCE<SIDECRITICALDISTANCE) {   //Obstacles detected on right of ELVIS
                if (RSpeed<0&&LSpeed>0) {   //If ELVIS is about to spin rightwards
                    LSpeed=0;
                    RSpeed=0;
                    MOTOR_WriteLSpeed(LSpeed);
                    MOTOR_WriteRSpeed(RSpeed);
                }
            }
            if (SONAR2DISTANCE<FRONTCRITICALDISTANCE||SONAR3DISTANCE<FRONTCRITICALDISTANCE) { //Obstacles detected on front of ELVIS
                SONAR_MaxMotorSpeed=CRITICALSPEED;
                MOTOR_WriteLSpeed(LSpeed);
                MOTOR_WriteRSpeed(RSpeed);
            }  else {
                SONAR_MaxMotorSpeed=MAXMOTORSPEED;
            }
            break;

    }
}

void MOTOR_Monitor()
{
    DataR[0] = (char)(0x00);
    DataR[1] = (char)(0xDC);
    DataR[2] = (char)(0x00);
    DataR[3] = (char)(0xE1);
    DataR[4] = (char)(0x00);
    DataR[5] = (char)(0x00);
    DataR[6] = (char)(0x00);
    DataR[7] = (char)(0x00);
    MOTOR_CANBus.write(CANMessage(CANID_RightMotor,DataR,8));   //Writes motor speed packet to CAN ID of right motor
}

int MOTOR_GetBatteryVoltage()
{
    return BatteryVoltage ;
}

void MOTOR_Test()
{
//Testing of Left Motor
    printf("TESTING OF LEFT MOTOR\n\r");
    wait(TESTWAITINTERVAL);
    printf("LEFT MOTOR FORWARD ROTATION FROM 0RPM TO MAXSPEED \n\r");
    LSpeed=MAXMOTORSPEED;
    wait(5);
    printf("LEFT MOTOR FORWARD ROTATION COMPLETED \n\r");
    wait(TESTWAITINTERVAL);
    printf("LEFT MOTOR REVERSE ROTATION FROM 0RPM TO -MAXSPEED \n\r");
    LSpeed=-MAXMOTORSPEED;
    wait(5);
    LSpeed=0;
    printf("LEFT MOTOR REVERSE ROTATION COMPLETED \n\r");
    wait(TESTWAITINTERVAL);
//Testing of Right Motor
    printf("TESTING OF RIGHT MOTOR\n\r");
    wait(TESTWAITINTERVAL);
    printf("RIGHT MOTOR FORWARD ROTATION FROM 0RPM TO MAXSPEED \n\r");
    RSpeed=MAXMOTORSPEED;
    wait(5);
    printf("RIGHT MOTOR FORWARD ROTATION COMPLETED \n\r");
    wait(TESTWAITINTERVAL);
    printf("RIGHT MOTOR REVERSE ROTATION FROM 0RPM TO -MAXSPEED \n\r");
    RSpeed=-MAXMOTORSPEED;
    wait(5);
    RSpeed=0;
    printf("RIGHT MOTOR REVERSE ROTATION COMPLETED \n\r");
    wait(TESTWAITINTERVAL);
}


