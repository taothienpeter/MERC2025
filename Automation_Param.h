#ifndef AUTOMATION_PARRAM_H
#define AUTOMATION_PARRAM_H
#include <Arduino.h>

class Semi_Autonomous {
private:
  double timing;
public:
  Semi_Autonomous();  // sets things, init pins, sensors, motors for class usage
  void Calculate_Timing();
  void Calculate_PID();
  void Drivetrain_Kinematic_Compute();
  double P, I, D;
protected:
};
#endif
