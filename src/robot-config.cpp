#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

//Global brain instance
brain Brain;
controller Controller1;

//Drive motors. 11 Watt motors
motor GearboxRF = motor(PORT5, ratio18_1, false);
motor GearboxRB = motor(PORT10, ratio18_1, true);

motor GearboxLF = motor(PORT14, ratio18_1, true);
motor GearboxLB = motor(PORT17, ratio18_1, false);

//5.5 Watt Drive motors
motor RightPush = motor(PORT8, ratio18_1, false);
motor LeftPush = motor(PORT21, ratio18_1, true);

motor Intake = motor(PORT15, ratio18_1, false);
motor kickerArm = motor(PORT11, ratio36_1, false);
motor kicker = motor(PORT12, ratio18_1, true);
inertial gyroscope = inertial(PORT8);
digital_out wings = digital_out(Brain.ThreeWirePort.C);
digital_out armHold = digital_out(Brain.ThreeWirePort.A);

void vexcodeInit(void) {
  // Nothing to initialize
}