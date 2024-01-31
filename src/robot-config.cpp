#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

//Global brain instance
brain Brain;
controller Controller1;

//Drive motors. 11 Watt motors
motor GearboxLF = motor(PORT15, ratio18_1, true);
motor GearboxLB = motor(PORT18, ratio18_1, false);

motor GearboxRF = motor(PORT6, ratio18_1, false);
motor GearboxRB = motor(PORT10, ratio18_1, true);

//5.5 Watt Drive motors
motor LeftPush = motor(PORT20, ratio18_1, true);
motor RightPush = motor(PORT2, ratio18_1, false);

motor Intake = motor(PORT21, ratio18_1, false);
motor kickerArm = motor(PORT12, ratio18_1, false);
motor kicker = motor(PORT2, ratio18_1, true);
inertial gyroscope = inertial(PORT11);
digital_out wings = digital_out(Brain.ThreeWirePort.A);

void vexcodeInit(void) {
  // Nothing to initialize
}