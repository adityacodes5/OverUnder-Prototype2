#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

//Global brain instance
brain Brain;
controller Controller1;

//Global competition objects instance. **ALL PORTS TO BE DEFINED LATER**
//Drive motors. 11 Watt motors
motor GearboxRF = motor(PORT22, ratio18_1, false);
motor GearboxRB = motor(PORT22, ratio18_1, true);

motor GearboxLF = motor(PORT22, ratio18_1, true);
motor GearboxLB = motor(PORT22, ratio18_1, false);

//5.5 Watt Drive motors
motor RightPush = motor(PORT22, ratio18_1, true);
motor LeftPush = motor(PORT22, ratio18_1, false);

motor Intake = motor(PORT22, ratio18_1, false);
motor kickerArm = motor(PORT22, ratio18_1, false);
inertial InertialSensor = inertial(PORT22);

void vexcodeInit(void) {
  // Nothing to initialize
}