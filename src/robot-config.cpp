#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

//Global brain instance
brain Brain;
controller Controller1;

//Drive motors. 11 Watt motors
motor GearboxRF = motor(PORT17, ratio18_1, false);
motor GearboxRB = motor(PORT16, ratio18_1, true);

motor GearboxLF = motor(PORT19, ratio18_1, true);
motor GearboxLB = motor(PORT20, ratio18_1, false);

//5.5 Watt Drive motors
motor RightPush = motor(PORT3, ratio18_1, true);
motor LeftPush = motor(PORT9, ratio18_1, false);

motor Intake = motor(PORT18, ratio18_1, false);
motor kickerArm = motor(PORT2, ratio18_1, false);
motor kiicker = motor(PORT10, ratio18_1, false);
inertial gyroscope = inertial(PORT1);

void vexcodeInit(void) {
  // Nothing to initialize
}