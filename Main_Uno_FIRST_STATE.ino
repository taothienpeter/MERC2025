#include "PS2X_lib.h"
#include <Wire.h>
#include "Manual_control.h"
#include "Automation_Param.h"

PS2X ps2_drivetrain;
PS2X ps2_arm;

int error = 0;    
int error_2 = 0;
byte type = 0;
byte vibrate = 0;
String GC_cmd;
short MotorSpeed[4], MotorPositions[4], MotorAcceleration, DirSpeed, DirAngle;
typedef enum RobotDir { FORWARD,
                BACKWARD,
                LEFTWARD,
                RIGHTWARD,
                CLOCKWISE,
                C_CLOCKWISE } RobotDir;

void setup() {
  Wire.begin(1); // join I2C bus on address #1
  Serial.begin(9600);
  error = ps2_drivetrain.config_gamepad(13,12,10,11, true, true); //(clock, command, attention, data, Pressures?, Rumble?)
  error = ps2_arm.config_gamepad(13,12,9,11, true, true);
}
void loop() {
  if (error) {
    delay(20);
    error = ps2_drivetrain.config_gamepad(13,12,10,11, true, true);
    error = ps2_arm.config_gamepad(13,12,9,11, true, true);
    return;}

  

  ps2_drivetrain.read_gamepad(false, vibrate);
  ps2_arm.read_gamepad(false, vibrate);
  if (ps2_drivetrain.Button(PSB_PAD_UP)) {
    Drivetrain_Kinematic_Compute(MotorSpeed, DirSpeed, FORWARD);
    Wire.write(GcodeGenerator(GC_cmd = "G1 V1 " + String(MotorSpeed[0]) +" V2 " + String(MotorSpeed[1]) +" V3 "+ String(MotorSpeed[2]) +" V4 "+ String(MotorSpeed[3])));
  }
  if (ps2_drivetrain.Button(PSB_PAD_DOWN)){
  }
  if (ps2_drivetrain.Button(PSB_PAD_RIGHT)){

  }
  if (ps2_drivetrain.Button(PSB_PAD_LEFT)){

  }
  
  delay(10);
}