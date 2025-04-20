#include "HardwareSerial.h"
// #include "HardwareSerial.h"
// #include "WString.h"
#include "Manual_control.h"
#include <Arduino.h>
#include <math.h>

// Semi_Autonomous SA;
// short MotorSpeed[4], MotorPositions[4], MotorAcceleration, DirSpeed, DirAngle;  // Motor names, driving Speed and driving Angle
char GC_cmd_type[6] = { 'M', 'V', 'A', 'P', 'I', 'D' };
double P[4], I[4], D[4];
// typedef enum GC_cmd_type {M, V, A, P, I, D} GC_cmd_type; // Pos(1, 2, 3, 4), vel, acc, Kp, Ki, Kd
extern  enum MotorName { MOTOR_LF,
                 MOTOR_RF,
                 MOTOR_LB,
                 MOTOR_RB } MotorName;
#ifdef DIRECT_CONTROL
extern typedef enum RobotDir { FORWARD,
                BACKWARD,
                LEFTWARD,
                RIGHTWARD,
                CLOCKWISE,
                C_CLOCKWISE } RobotDir;
#endif
void Drivetrain_Kinematic_Compute(short* MotorSpeed[4], short DirSpeed, RobotDir DirAngle) {
  #ifdef KINEMATIC_CONTROL
    int theta;
    int sin = sin(theta - M_PI / 4);
    int cos = cos(theta - M_PI / 4);
  #endif  // KINEMATIC_CONTROL
  #ifdef DIRECT_CONTROL
    switch (DirAngle) {
      case FORWARD:
        *MotorSpeed = DirSpeed;
        *(MotorSpeed + 1) = DirSpeed;
        *(MotorSpeed + 2) = -DirSpeed;
        *(MotorSpeed + 3) = -DirSpeed;
        break;
      case BACKWARD:
        *MotorSpeed = -DirSpeed;
        *(MotorSpeed + 1) = -DirSpeed;
        *(MotorSpeed + 2) = DirSpeed;
        *(MotorSpeed + 3) = DirSpeed;
        break;
      case LEFTWARD:
        *MotorSpeed = -DirSpeed;
        *(MotorSpeed + 1) = DirSpeed;
        *(MotorSpeed + 2) = DirSpeed;
        *(MotorSpeed + 3) = -DirSpeed;
        break;
      case RIGHTWARD:
        *MotorSpeed       = DirSpeed;
        *(MotorSpeed + 1) = -DirSpeed;
        *(MotorSpeed + 2) = -DirSpeed;
        *(MotorSpeed + 3) = DirSpeed;
        break;
      case CLOCKWISE:
        *MotorSpeed       = DirSpeed;
        *(MotorSpeed + 1) = -DirSpeed;
        *(MotorSpeed + 2) = DirSpeed;
        *(MotorSpeed + 3) = -DirSpeed;
        break;
      case C_CLOCKWISE:
        *MotorSpeed       = -DirSpeed;
        *(MotorSpeed + 1) = DirSpeed;
        *(MotorSpeed + 2) = -DirSpeed;
        *(MotorSpeed + 3) = DirSpeed;
        break;
    }
  #endif  // DIRECT_CONTROL
}
void SerialGcode() {
}
void I2C_com(){

}
void GC_decode(String* GC_return, int spaceIndex, String token, String GC_cmd, short gct, short input_array[4]) {
  char motorIndexChar = token.charAt(1);
  if (motorIndexChar >= '1' && motorIndexChar <= '4') {
    int motorIndex = motorIndexChar - '1';  // Convert char to index (0-3)
    // Get the value after "M<index>"
    spaceIndex = GC_cmd.indexOf(' ');
    String valueStr;
    if (spaceIndex == -1) {
      valueStr = GC_cmd;
      GC_cmd = "";
    } else {
      valueStr = GC_cmd.substring(0, spaceIndex);
      GC_cmd = GC_cmd.substring(spaceIndex + 1);
      GC_cmd.trim();
    }
    int value = valueStr.toInt();
    if (value != 0 || valueStr == "0") {  // Check if it's a valid number
      input_array[motorIndex] = value;
    } else {
      Serial.print("Invalid value for motor ");
      Serial.println(GC_cmd_type[gct]);
      return;
    }
  } else {
    for (int i = 0; i < 4; i++) {
      *GC_return += " " + GC_cmd_type[gct] + String(i + 1) + " " + String(input_array[i]);
    }
  }
}
void GC_decode_parram(String* GC_return, int spaceIndex, String token, String GC_cmd, short gct, double input_parram[4]) {
  char motorIndexChar = token.charAt(1);
  if (motorIndexChar >= '1' && motorIndexChar <= '4') {
    int motorIndex = motorIndexChar - '1';  // Convert char to index (0-3)
    // Get the value after "M<index>"
    spaceIndex = GC_cmd.indexOf(' ');
    String valueStr;
    if (spaceIndex == -1) {
      valueStr = GC_cmd;
      GC_cmd = "";
    } else {
      valueStr = GC_cmd.substring(0, spaceIndex);
      GC_cmd = GC_cmd.substring(spaceIndex + 1);
      GC_cmd.trim();
    }
    int value = valueStr.toInt();
    if (value != 0 || valueStr == "0") {  // Check if it's a valid number
    *GC_return += " " + GC_cmd_type[gct] + String(motorIndex) + " " + String(input_parram[motorIndex-1]);
      // input_parram[motorIndex] = value;
    } else {
      Serial.print("Invalid value for motor ");
      Serial.println(GC_cmd_type[gct]);
      return;
    }
  } else {
    for (int i = 0; i < 4; i++) {
      *GC_return += " " + GC_cmd_type[gct] + String(i + 1) + " " + String(input_parram[i]);
    }
  }
}
String GcodeGenerator(String GC_cmd) {
    String GC_return = "";
    if (GC_cmd.startsWith("G1")) {  // RUNNING mode for Pico 1 aka DRIVETRAIN
      GC_return = "G1";
      //======
      int spaceIndex;
      String token;
      GC_cmd = GC_cmd.substring(2);
      GC_cmd.trim();
      while (GC_cmd.length() > 0) {
        spaceIndex = GC_cmd.indexOf(' ');
        if (spaceIndex == -1) {
          token = GC_cmd;
          GC_cmd = "";
        } else {
          token = GC_cmd.substring(0, spaceIndex);
          GC_cmd = GC_cmd.substring(spaceIndex + 1);
          GC_cmd.trim();
        }  // tách cmd
        //==========
        if (token.startsWith("M")) {
  #ifdef GCODE_POS_CMD  // String* GC_return, int* spaceIndex, String* token, String* GC_cmd, short input_array[Outputindex]
          GC_decode(&GC_return, spaceIndex, token, GC_cmd, 0, MotorPositions);
          // char motorIndexChar = token.charAt(1);
          // if (motorIndexChar >= '1' && motorIndexChar <= '4') {
          //       int motorIndex = motorIndexChar - '1'; // Convert char to index (0-3)
          //       // Get the value after "M<index>"
          //       spaceIndex = GC_cmd.indexOf(' ');
          //       String valueStr;
          //       if (spaceIndex == -1) {
          //             valueStr = GC_cmd;
          //             GC_cmd = "";
          //       } else {
          //             valueStr = GC_cmd.substring(0, spaceIndex);
          //             GC_cmd = GC_cmd.substring(spaceIndex + 1);
          //             GC_cmd.trim();
          //       }
          //       int value = valueStr.toInt();
          //       if (value != 0 || valueStr == "0") { // Check if it's a valid number
          //             motorPositions[motorIndex] = value;
          //       } else {
          //             Serial.println("Invalid value for motor position.");
          //       break;
          //       }
          // } else {
          //       for(int i =0; i<4; i++){
          //             GC_return += " M" + String(i+1) + " " + String(MotorPositions[i]);
          //       }
          // }
  #endif  // GCODE_POS_CMD
        } else if (token.startsWith("V")) {
  #ifdef GCODE_VEL_CMD
          GC_decode(&GC_return, spaceIndex, token, GC_cmd, 1, MotorSpeed);
            // char motorIndexChar = token.charAt(1);
            // if (motorIndexChar >= '1' && motorIndexChar <= '4') {
            //       int motorIndex = motorIndexChar - '1'; // Convert char to index (0-3)
            //       // Get the value after "M<index>"
            //       spaceIndex = GC_cmd.indexOf(' ');
            //       String valueStr;
            //       if (spaceIndex == -1) {
            //             valueStr = GC_cmd;
            //             GC_cmd = "";
            //       } else {
            //             valueStr = GC_cmd.substring(0, spaceIndex);
            //             GC_cmd = GC_cmd.substring(spaceIndex + 1);
            //             GC_cmd.trim();
            //       }
            //       int value = valueStr.toInt();
            //       if (value != 0 || valueStr == "0") { // Check if it's a valid number
            //             motorPositions[motorIndex] = value;
            //       } else {
            //             Serial.println("Invalid value for motor velocity.");
            //       break;
            //       }
            // } else {
            //       for(int i = 0; i<4; i++){
            //             GC_return += " V" + String(i+1) + " " + String(MotorSpeeds[i]);
            //       }
            // }
  #endif  // GCODE_VEL_CMD
        } else if (token.startsWith("A")) {
  #ifdef GCODE_ACC_CMD
          for (int i = 0; i < 4; i++) {
            GC_return += " A" + String(i + 1) + " " + String(MotorAcceleration);
          }
  #endif  // GCODE_ACC_CMD
        } else {
          Serial.print("Done generate Gcode for pico 1 MOTORS RUNNING");
        }
      }
      // switch (GC_cmd) {
      //       case M:
      //             #ifdef GCODE_POS_CMD
      //                   for(int i =0; i<4; i++){
      //                         GC_return += " M" + String(i+1) + " " + String(MotorPositions[i]);
      //                   }
      //             #endif // GCODE_POS_CMD
      //       case V:
      //             #ifdef GCODE_VEL_CMD
      //                   for(int i = 0; i<4; i++){
      //                         GC_return += " V" + String(i+1) + " " + String(MotorSpeeds[i]);
      //                   }
      //             #endif // GCODE_VEL_CMD
      //       case A:
      //             #ifdef GCODE_ACC_CMD
      //                   for(int i= 0; i<4; i++){
      //                         GC_return += " A" + String(i+1) + " " + String(MotorAcceleration);
      //                   }
      //             #endif // GCODE_ACC_CMD
      // default:
      //       Serial.print("Done generate Gcode for MOTORS");
      // }
    } else if (GC_cmd.startsWith("M1")) {  // SETTING mode for Pico 1 aka DRIVETRAIN
      GC_return = "M1";
  #ifdef PID_CONFIG
      int spaceIndex;
      String token;
      GC_cmd = GC_cmd.substring(2);
      GC_cmd.trim();
      while (GC_cmd.length() > 0) {
        spaceIndex = GC_cmd.indexOf(' ');
        if (spaceIndex == -1) {
          token = GC_cmd;
          GC_cmd = "";
        } else {
          token = GC_cmd.substring(0, spaceIndex);
          GC_cmd = GC_cmd.substring(spaceIndex + 1);
          GC_cmd.trim();
        }  // tách cmd
        //==========
        if (token.startsWith("P")) {
          GC_decode_parram(&GC_return, spaceIndex, token, GC_cmd, 3, P); //======================
            // char motorIndexChar = token.charAt(1);
            // if (motorIndexChar >= '1' && motorIndexChar <= '4') {
            //       int motorIndex = motorIndexChar - '1'; // Convert char to index (0-3)
            //       // Get the value after "M<index>"
            //       spaceIndex = GC_cmd.indexOf(' ');
            //       String valueStr;
            //       if (spaceIndex == -1) {
            //             valueStr = GC_cmd;
            //             GC_cmd = "";
            //       } else {
            //             valueStr = GC_cmd.substring(0, spaceIndex);
            //             GC_cmd = GC_cmd.substring(spaceIndex + 1);
            //             GC_cmd.trim();
            //       }
            //       int value = valueStr.toInt();
            //       if (value != 0 || valueStr == "0") { // Check if it's a valid number
            //             motorPositions[motorIndex] = value;
            //       } else {
            //             Serial.println("Invalid value for Kp parameter.");
            //       break;
            //       }
            // } else {
            //       for(int i =0; i<4; i++){
            //             GC_return += " P" + String(i+1) + " " + String(MotorPositions[i]);
            //       }
            // }
        } else if (token.startsWith("I")) {
          GC_decode_parram(&GC_return, spaceIndex, token, GC_cmd, 3, I);
            // char motorIndexChar = token.charAt(1);
            // if (motorIndexChar >= '1' && motorIndexChar <= '4') {
            //   int motorIndex = motorIndexChar - '1';  // Convert char to index (0-3)
            //   // Get the value after "M<index>"
            //   spaceIndex = GC_cmd.indexOf(' ');
            //   String valueStr;
            //   if (spaceIndex == -1) {
            //     valueStr = GC_cmd;
            //     GC_cmd = "";
            //   } else {
            //     valueStr = GC_cmd.substring(0, spaceIndex);
            //     GC_cmd = GC_cmd.substring(spaceIndex + 1);
            //     GC_cmd.trim();
            //   }
            //   int value = valueStr.toInt();
            //   if (value != 0 || valueStr == "0") {  // Check if it's a valid number
            //     GC_return += " I" + String
            //   } else {
            //     Serial.println("Invalid value for Kp parameter.");
            //     break;
            //   }
            // } else {
            //   for (int i = 0; i < 4; i++) {
            //     GC_return += " I" + String(i + 1) + " " + String(I);
            //   }
            // }
        } else if (token.startsWith("D")) {
          GC_decode_parram(&GC_return, spaceIndex, token, GC_cmd, 3, D);
            // char motorIndexChar = token.charAt(1);
            // if (motorIndexChar >= '1' && motorIndexChar <= '4') {
            //   int motorIndex = motorIndexChar - '1';  // Convert char to index (0-3)
            //   // Get the value after "M<index>"
            //   spaceIndex = GC_cmd.indexOf(' ');
            //   String valueStr;
            //   if (spaceIndex == -1) {
            //     valueStr = GC_cmd;
            //     GC_cmd = "";
            //   } else {
            //     valueStr = GC_cmd.substring(0, spaceIndex);
            //     GC_cmd = GC_cmd.substring(spaceIndex + 1);
            //     GC_cmd.trim();
            //   }
            //   int value = valueStr.toInt();
            //   if (value != 0 || valueStr == "0") {  // Check if it's a valid number
            //     GC_return += " D" + String
            //   } else {
            //     Serial.println("Invalid value for Kp parameter.");
            //     break;
            //   }
            // } else {
            //   for (int i = 0; i < 4; i++) {
            //     GC_return += " D" + String(i + 1) + " " + String(D);
            //   }
          }
          else {
            Serial.print("Done generate Gcode for pico 1 MOTORS SETTING");
          }
        }
  #endif
      }
      else if (GC_cmd.startsWith("G2")) {
        GC_return = "G2";
      }
      else if (GC_cmd.startsWith("M2")) {
        GC_return = "M2";
      }
      else {
        GC_return = "G0";
      };
      return GC_return += '\0';
}
