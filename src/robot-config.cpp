#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

//Global brain instance
brain Brain;
controller Controller1;

//Drive motors. 11 Watt motors
motor GearboxRF = motor(PORT9, ratio18_1, false);
motor GearboxRB = motor(PORT12, ratio18_1, true);

motor GearboxLF = motor(PORT17, ratio18_1, true);
motor GearboxLB = motor(PORT16, ratio18_1, false);

//5.5 Watt Drive motors
motor RightPush = motor(PORT8, ratio18_1, false);
motor LeftPush = motor(PORT21, ratio18_1, true);

motor Intake = motor(PORT11, ratio18_1, false);
motor kickerArm = motor(PORT10, ratio18_1, false);
motor kicker = motor(PORT2, ratio36_1, false);
inertial gyroscope = inertial(PORT1);
digital_out wings = digital_out(Brain.ThreeWirePort.A);

void vexcodeInit(void) {
  // Nothing to initialize
}