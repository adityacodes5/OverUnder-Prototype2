using namespace vex;

extern brain Brain;

extern drivetrain Drivetrain;
extern controller Controller1;

//Drive motors. 11 Watt
extern motor GearboxRF;
extern motor GearboxRB;
extern motor GearboxLF;
extern motor GearboxLB;
//5.5 Watt
extern motor RightPush;
extern motor LeftPush;

extern motor Intake;
extern motor kickerArm;

extern inertial InertialSensor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
