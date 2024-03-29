extern double overrideMovement(double value1, double value2);
extern void gearboxL(directionType direction, double velocity);
extern void gearboxR(directionType direction, double velocity);
extern void move(directionType direction, double velocityL, double velocityR);
extern void moveUntil(double degrees, double motorVelocity, bool waitForCompletion);
extern void brakeDrive(brakeType driveBrake);
extern double calculateAverageMotorRotation();
extern void resetDegreePosition();
extern double inchesToDegrees(double inchesToTurn);
extern double degreesToInches(double degreesToTurn);
extern double measureDistance(int miliSeconds);
extern void kick(double velocity, bool pullBack);
extern void moveInches(double inchesToTurn, double motorVelocity, bool waitForCompletion, bool forward);
extern void moveInchesPID(double inchesToTurn, double motorVelocity, bool forward);
extern void oldMoveInches(double inchesToTurn, double motorVelocity);
extern void ProgSkills();

extern double timeElapsed(timeUnits units);
extern void resetTimer();
extern double getHeading();
extern void resetHeading();
extern double getRoll();
extern void pregameCalibrate();
extern bool autoDrive();
extern void grabTriball(double velocity_mV, bool outTake);
extern void moveArm();
extern void alertRumble(bool rapidFire);
extern void startTurn(double Target, double settleDist, double p, double i, double d);
extern void matchLoadAuton();
extern void closeSideAuton();
extern void setWings();
extern void newTurn(double targetAngle, double speed);
extern void newMoveInches(int inchesToMove,int multiplierSpeed);

