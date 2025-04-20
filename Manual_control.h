#include "WString.h"
#ifndef MANUAL_CONTROL_H
#define MANUAL_CONTROL_H
#include <Arduino.h>
// #include "Automation_Param.h"



// #define KINEMATIC_CONTROL // use for kinematic computing
#define DIRECT_CONTROL // use for manual controller only
// #define GCODE_POS_CMD // enable position Gcode generation
#define GCODE_VEL_CMD // enable velocity Gcode generation
// #define GCODE_ACC_CMD // enable accelleration Gcode generation
// #define PID_CONFIG  // enable PID config Gcode generation

extern short MotorSpeed[4], MotorPositions[4], MotorAcceleration, DirSpeed, DirAngle;  // Motor names, driving Speed and driving Angle
// extern char GC_cmd_type[6] = { 'M', 'V', 'A', 'P', 'I', 'D' };
extern double P[4], I[4], D[4]; // storing PID varable for 4 motors
extern String GC_cmd; // storing input Gcode command for Gcode gen M1, M2, G1, G2
void SerialGcode(); // Serial Gcode command user input
void GC_decode(String* GC_return, int spaceIndex, String token, String GC_cmd, short gct, short input_array[4]); // Gcode small cmd for operating
void GC_decode_parram(String* GC_return, int spaceIndex, String token, String GC_cmd, short gct, double input_parram[4]); //// Gcode small cmd for parram
String GcodeGenerator(String GC_cmd); // for Gcode generation
void Drivetrain_Kinematic_Compute(short MotorSpeed[4], short DirSpeed, short* RobotDir); // compute Drivetrain kinematic
void Com_I2C();
#endif