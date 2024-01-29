#include "vex.h"
//#include <iostream>
using namespace std;
vex::timer myTimer; //Create a timer for the program

const double wheelCircumference = 4 * M_PI; //Circumference of the wheel
const double gearRatio = 1.5; //Gear ratio of the drive
const int percentToMVolts = 120;
const int deltaTime = 10; //Time between each loop of the PID loop
const int maxVoltage = 12000; //Max voltage of the motor
const int delayTime = 10;

bool accelerationCompleted;
bool runCompleted;
bool decelerationCompleted;

double currentRotationBackL;
double currentRotationMiddleL;
double currentRotationFrontL; 

double currentRotationBackR;
double currentRotationMiddleR;
double currentRotationFrontR;

double averageRotation;
double degreesToTurn;
double distanceInches;
double averageDriveRotation;
double distanceDegrees;
double distanceError;
double distanceErrorToTurn;
double distanceErrorDerivative;
double baseVelocityRatio;
double errorVelocity;

//turn variables
double kP;
double kI;
double kD;
double error;
double prevError;
double targetAngle;
double currentOrientation;
double integral;
double derivative;
double speedL;
double speedR;
double settlingDist;
bool leftTurn;
bool enableTurn;

double valueRotation;

double leftAxisPCT;
double leftAxismV;
double rightAxisPCT;
double rightAxismV;

double tankAxisPCT;
double tankAxismV;

double armAxisPCT;
double armAxismV;

double rightSpeed, leftSpeed;

double overrideMovement(double value1, double value2){ //Return greater value of two numbers (absolute value). Used for allowing only one stick to work at a time, taking the one with the greater value

   return abs(value1) > abs(value2) ? value1 : value2;

}

void gearboxL(directionType direction, double velocity){ //Movement group for just the left Gearbox

    GearboxLF.spin(direction, velocity, velocityUnits::pct);
    GearboxLB.spin(direction, velocity, velocityUnits::pct);

}

void gearboxR(directionType direction, double velocity){ //Movement group for just the right Gearbox

    GearboxRF.spin(direction, velocity, velocityUnits::pct);
    GearboxRB.spin(direction, velocity, velocityUnits::pct);

}

void move(directionType direction, double velocityL, double velocityR){ //Movement group for entire drive

    GearboxLF.spin(direction, velocityL, velocityUnits::pct);
    GearboxLB.spin(direction, velocityL, velocityUnits::pct);
    GearboxRF.spin(direction, velocityR, velocityUnits::pct);
    GearboxRB.spin(direction, velocityR, velocityUnits::pct);

    RightPush.spin(direction, velocityR, velocityUnits::pct);
    LeftPush.spin(direction, velocityL, velocityUnits::pct);

}


void moveUntil(double degrees, double motorVelocity, bool waitForCompletion){ //Move Drive until a certain degree

    GearboxRF.spinTo(degrees, rotationUnits::deg, motorVelocity, velocityUnits::pct, waitForCompletion);
    GearboxRB.spinTo(degrees, rotationUnits::deg, motorVelocity, velocityUnits::pct, waitForCompletion);

    GearboxLF.spinTo(degrees, rotationUnits::deg, motorVelocity, velocityUnits::pct, waitForCompletion);
    GearboxLB.spinTo(degrees, rotationUnits::deg, motorVelocity, velocityUnits::pct, waitForCompletion);

    RightPush.spinTo(degrees, rotationUnits::deg, motorVelocity, velocityUnits::pct, waitForCompletion);
    LeftPush.spinTo(degrees, rotationUnits::deg, motorVelocity, velocityUnits::pct, waitForCompletion);

}

void brakeDrive(brakeType driveBrake){ //Brake group for entire drive

    GearboxRF.stop(driveBrake);
    GearboxRB.stop(driveBrake);

    GearboxLF.stop(driveBrake);
    GearboxLB.stop(driveBrake);

    RightPush.stop(driveBrake);
    LeftPush.stop(driveBrake);

}

double calculateAverageMotorRotation(){

    valueRotation = (

        GearboxRF.position(rotationUnits::deg) +
        GearboxRB.position(rotationUnits::deg) +

        GearboxLF.position(rotationUnits::deg) +
        GearboxLB.position(rotationUnits::deg) +

        RightPush.position(rotationUnits::deg) +
        LeftPush.position(rotationUnits::deg) 

        ) / 6;

    return valueRotation;

}

void oldMoveInches(double inchesToTurn, double motorVelocity){
    GearboxLB.resetPosition(); //Reset the back left motor's rotation
    GearboxLF.resetPosition(); //Reset the middle left motor's rotation
    GearboxRB.resetPosition(); //Reset the front left motor's rotation

    GearboxRF.resetPosition(); //Reset the back right motor's rotation
    LeftPush.resetPosition(); //Reset the middle right motor's rotation
    RightPush.resetPosition(); //Reset the front right motor's rotation

    int i;

    
    averageRotation = 0; //Declare average rotation variable
    degreesToTurn = (inchesToTurn / wheelCircumference) * (360/gearRatio); //Find the number of degrees to turn by dividing the inches to turn by the circumference of the wheel and multiplying by 360. Use negative distance and velocity if going backward

    while (fabs(RightPush.position(rotationUnits::deg)) <= fabs(degreesToTurn)){//*NOTE*: change degreesToTurn back to averageRotation when all the motor values have the same sign
        currentRotationBackL = GearboxLB.position(rotationUnits::deg); //Get the current rotation of the back left motor
        currentRotationMiddleL = GearboxLF.position(rotationUnits::deg); //Get the current rotation of the middle left motor
        currentRotationFrontL = GearboxRB.position(rotationUnits::deg); //Get the current rotation of the front left motor

        currentRotationBackR = GearboxRF.position(rotationUnits::deg); //Get the current rotation of the back right motor
        currentRotationMiddleR = LeftPush.position(rotationUnits::deg); //Get the current rotation of the middle right motor
        currentRotationFrontR = RightPush.position(rotationUnits::deg); //Get the current rotation of the front right motor

        averageRotation = (currentRotationBackL + currentRotationMiddleL + currentRotationFrontL + currentRotationBackR + currentRotationMiddleR + currentRotationFrontR) / 6; //Find the average rotation of all 6 motors

        GearboxLB.spinTo(degreesToTurn, rotationUnits::deg, motorVelocity, velocityUnits::pct, false);
        GearboxLF.spinTo(degreesToTurn, rotationUnits::deg, motorVelocity, velocityUnits::pct, false);
        LeftPush.spinTo(degreesToTurn, rotationUnits::deg, motorVelocity, velocityUnits::pct, false);

        GearboxRB.spinTo(degreesToTurn, rotationUnits::deg, motorVelocity, velocityUnits::pct, false);
        GearboxRF.spinTo(degreesToTurn, rotationUnits::deg, motorVelocity, velocityUnits::pct, false);
        RightPush.spinTo(degreesToTurn, rotationUnits::deg, motorVelocity, velocityUnits::pct, false);


        if (RightPush.position(rotationUnits::deg) > degreesToTurn){
            brakeDrive(coast);
            break;
        }
        
    }
}

void moveInches(double inchesToTurn, double motorVelocity, bool waitForCompletion, bool forward){ //Move Drive a set amount of inches.

    degreesToTurn = inchesToDegrees(inchesToTurn);

    if (forward){
        degreesToTurn = degreesToTurn * -1;
    }

    while(calculateAverageMotorRotation() < degreesToTurn){

        moveUntil(degreesToTurn, motorVelocity, true);

        if(calculateAverageMotorRotation() >= degreesToTurn){

            brakeDrive(coast);
            break;

        }
    }

}

void resetDegreePosition(){

    GearboxRF.resetPosition();
    GearboxRB.resetPosition();

    GearboxLF.resetPosition();
    GearboxLB.resetPosition();

    RightPush.resetPosition();
    LeftPush.resetPosition();

}

double inchesToDegrees(double inchesToTurn){

    degreesToTurn = (inchesToTurn/wheelCircumference) * (360/gearRatio); //Convert inches to degrees
//    std::cout << degreesToTurn << std::endl;
    return degreesToTurn;

}

double degreesToInches(double degreesMoved){
    
    distanceInches = (degreesMoved * gearRatio * wheelCircumference) / 360; //Convert degrees to inches
    return distanceInches;
    
    
}

double measureDistance(int miliSeconds){
    
    resetDegreePosition();
    wait(miliSeconds,msec);
    return degreesToInches(calculateAverageMotorRotation());

}


void moveInchesPID(double inchesToTurn, double motorVelocity, bool forward){ //Move Drive a set amount of inches.

    resetDegreePosition(); //Set all degree measurements to 0

    distanceDegrees = inchesToDegrees(inchesToTurn);


    accelerationCompleted = false;
    runCompleted = false;
    decelerationCompleted = false;

    distanceErrorToTurn  = distanceDegrees/10; //Convert inches to degrees of the distance error. Divide by 10 to get first 10% of distance needed to cover to accelerate
    degreesToTurn = distanceDegrees*8/10; //Convert inches to degrees 
    distanceErrorDerivative = 0.01*distanceErrorToTurn; //Get derivative of distance error. Recalculate velocity every incrament of 1% or 0.01 of the distance error
    baseVelocityRatio = 100/motorVelocity; //Get denominator for calculating terminal velocity

    while((calculateAverageMotorRotation() < distanceErrorToTurn) && !accelerationCompleted){ //Beginning of PID loop, to accelerate the robot

        errorVelocity = 15*(log2(distanceErrorDerivative + 1)/ baseVelocityRatio); //Calculate velocity based on distance error derivative. Intented to hit Vertex (100, 100) on a graph
        moveUntil(distanceErrorToTurn, errorVelocity, false);
        distanceErrorDerivative += 0.01*distanceErrorToTurn; //Incrament distance error derivative by 1%

        if(calculateAverageMotorRotation() >= distanceErrorToTurn){

            accelerationCompleted = true;
            brakeDrive(coast);
            break;

        }

        
    }

    while((calculateAverageMotorRotation() < (degreesToTurn + distanceErrorToTurn)) && !runCompleted && accelerationCompleted){ //Linear velocity loop to maintain terminal velocity for set amount of degrees

        moveUntil(degreesToTurn, motorVelocity, true);
        
        if(calculateAverageMotorRotation() >= degreesToTurn){

            runCompleted = true;
            brakeDrive(coast);
            break;

        }

    }
    
    while((calculateAverageMotorRotation() < distanceDegrees) && !decelerationCompleted && runCompleted){ //Deceleration loop to slow down the robot

        errorVelocity = 15*(log2(distanceErrorDerivative + 1)/ baseVelocityRatio); //Calculate velocity based on distance error derivative. Intented to hit Vertex (100, 100) on a graph
        moveUntil(distanceDegrees, errorVelocity, false);
        distanceErrorDerivative -= 0.01*distanceErrorToTurn; //Decrement distance error derivative by 1%

        if(calculateAverageMotorRotation() >= distanceDegrees){

            decelerationCompleted = true;
            brakeDrive(coast);
            break;

        }

    }

    brakeDrive(hold);

}

double timeElapsed(timeUnits units){

    return myTimer.time(units);

}

void resetTimer(){
    
    myTimer.reset();

}

double getHeading(){
                    
    return gyroscope.heading();

}

double getRoll(){
        
    return gyroscope.roll();

}

void resetHeading(){
        
    gyroscope.setHeading(0.1, rotationUnits::deg); // 0.1 to avoid 0/360 glitch

}

void pregameCalibrate(){

    gyroscope.calibrate();
    

    while (gyroscope.isCalibrating()) {

        this_thread::sleep_for(100);

    }

}

bool autoDrive(){
    
    leftAxisPCT = Controller1.Axis3.position(percent) + Controller1.Axis4.position(percent);
    leftAxismV = leftAxisPCT * percentToMVolts;
    rightAxisPCT = Controller1.Axis3.position(percent) - Controller1.Axis4.position(percent);
    rightAxismV = rightAxisPCT * percentToMVolts;
    
//    tankAxisPCT = Controller1.Axis2.position(percent);
//    tankAxismV = tankAxisPCT * percentToMVolts;

//    rightSpeed = overrideMovement(rightAxismV, tankAxismV);
//    leftSpeed = overrideMovement(leftAxismV, tankAxismV);

//    if (abs(rightSpeed) > 5 || abs(leftSpeed) > 5){
//
//        move(fwd, leftSpeed, rightSpeed);
//
//    }
//
//    else{
//
//        brakeDrive(brake);
//
//    }


    if (abs(rightAxisPCT) > 5 || abs(leftAxisPCT) > 5){

        move(fwd, leftAxisPCT, rightAxisPCT);
        return true;

    }

    else{

        brakeDrive(brake);
        return false;

    }

    

}

void grabTriball(double velocity_mV, bool outTake){ //Spin intake to grab ball, or outtake ball from intake. Specify velcoity in mV (12000 max) and direction (in or out)

    if (!outTake){

        Intake.spin(fwd, velocity_mV, voltageUnits::mV);

    }

    else if (outTake){

        Intake.spin(reverse, velocity_mV, voltageUnits::mV);

    }
    
    else if (velocity_mV == 0) {

        Intake.stop(coast);

    }

}

void kick(double velocity_pct, bool pullBack){ //Spin kicker to kick ball, or "pullBack" (reverse) to fix if it is stuck

    kicker.setMaxTorque(100, percent);

    if (!pullBack){

        kicker.spin(fwd, velocity_pct, percentUnits::pct);

    }

    else if (pullBack){

        kicker.spin(reverse, velocity_pct, percentUnits::pct);

    }
    
    else if (velocity_pct == 0) {

        kicker.stop(coast);

    }

}

void moveArm(){

    armAxisPCT = Controller1.Axis2.position(percent);
    armAxismV = armAxisPCT * percentToMVolts;

    kickerArm.setMaxTorque(100,percentUnits::pct);

    if (abs(armAxisPCT) > 5){

        kickerArm.spin(fwd, armAxismV, voltageUnits::mV);

    }

    else{

        kickerArm.stop(hold);

    }

}

void alertRumble(bool rapidFire){

    if (rapidFire){

        Controller1.rumble("..-..");

    }

    else{

        Controller1.rumble("-.-.");

    }


}

void resetTurnVariables(){

    error = 0;
    prevError = 0;
    targetAngle = 0;
    currentOrientation = 0;
    integral = 0;
    derivative = 0;
    speedL = 0;
    speedR = 0;
    leftTurn = false;

}

void startTurn(double Target, double settleDist, double p, double i, double d){

    resetTurnVariables();
    resetHeading();
    targetAngle = Target;

    settlingDist = settleDist;

    kP = p;
    kI = i;
    kD = d;

    if (p == 0 && i == 0 && d == 0) {

        if(fabs(targetAngle) <= 90) {  //Small turn
            kP = 0.09;
            kI = 0;
            kD = 0;
        }

        else if(fabs(targetAngle) > 90 && targetAngle <= 150) { //Medium turn
            kP = 0.091;
            kI = 0;
            kD = 0;
        }

        else { // Long Turn
            kP = 0.08421;
            kI = 0;
            kD = 0;
        }

    if(targetAngle < 0) { 
        targetAngle = 360 - fabs(targetAngle);
        gyroscope.setHeading(359.9, rotationUnits::deg); // 359.9 to avoid 0/360 glitch
        leftTurn = true;
        }

    }

    enableTurn = true;

    while(enableTurn){
        
        currentOrientation = getHeading();

        error = targetAngle - currentOrientation;
        integral += error*deltaTime;
        derivative = (error - prevError)/deltaTime;
        prevError = error;

        speedL = (error*kP) + (integral*kI) + (derivative*kD);
        speedR = -1*((error*kP) + (integral*kI) + (derivative*kD));

        if(speedL > maxVoltage){
            speedL = copysign(maxVoltage, speedL);
            speedR = copysign(maxVoltage, speedR);
        }

        move(fwd, speedL, speedR);

        if(fabs(error) <= settlingDist){
            brakeDrive(hold);
            enableTurn = false;
            break;
        }

        vexDelay(deltaTime);
        
    }
}

void matchLoadAuton(){ //bot is set backwards
    move(directionType::rev, 72, 60);
    vexDelay(1500);
    brakeDrive(coast);
    wait(250, msec);
    move(directionType::fwd, 20, 20);
    wait(500, msec);
    move(directionType::rev, 80, 80);
    wait(1000,msec);
    move(fwd, 30, 30);
    wait(1500, msec);
    brakeDrive(coast);
    kickerArm.spinTo(540, rotationUnits::deg, true);
}

void closeSideAuton(){ //bot is set forwards
    move(fwd, 60, 72);
    vexDelay(1500);
    brakeDrive(coast);
    wait(500, msec);
    grabTriball(100*120, true);
    move(fwd, 20, 20);
    wait(1000, msec);
    grabTriball(0, true);
    brakeDrive(coast);
    wait(500, msec);   
    move(directionType::rev, 20, 20);
    wait(500, msec);
    move(fwd, 80, 80);
    wait(1000,msec);
    move(directionType::rev, 30, 30);
    wait(1000, msec);
    brakeDrive(coast);
}

void setWings(){
    if(wings.value() == 1){
        wings.set(false);
    }
    else{
        wings.set(true);
    }
}
