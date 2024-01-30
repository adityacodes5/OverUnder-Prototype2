#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

//Global brain instance
brain Brain;
controller Controller1;

//Drive motors. 11 Watt motors
motor GearboxLF = motor(PORT9, ratio18_1, true);
motor GearboxLB = motor(PORT6, ratio18_1, false);

motor GearboxRF = motor(PORT17, ratio18_1, false);
motor GearboxRB = motor(PORT15, ratio18_1, true);

//5.5 Watt Drive motors
motor LeftPush = motor(PORT3, ratio18_1, true);
motor RightPush = motor(PORT11, ratio18_1, false);

motor Intake = motor(PORT18, ratio18_1, false);
motor kickerArm = motor(PORT14, ratio18_1, false);
motor kicker = motor(PORT2, ratio36_1, false);
inertial gyroscope = inertial(PORT1);
digital_out wings = digital_out(Brain.ThreeWirePort.A);

void vexcodeInit(void) {
  // Nothing to initialize
}