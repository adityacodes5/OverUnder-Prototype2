#include "vex.h"
//#include <iostream>
using namespace std;
vex::timer myTimer; //Create a timer for the program

const double wheelCircumference = 4 * M_PI; //Circumference of the wheel
const double gearRatio = 1.5; //Gear ratio of the drive
const double percentToMVolts = 120;

bool accelerationCompleted;
bool runCompleted;
bool decelerationCompleted;

double degreesToTurn;
double distanceInches;
double averageDriveRotation;
double distanceDegrees;
double distanceError;
double distanceErrorToTurn;
double distanceErrorDerivative;
double baseVelocityRatio;
double errorVelocity;

double distanceRequired;
double kI;
double kP;
double kD;
double error;
double previousError;
double integral;
double derivative;



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

    gearboxL(direction, velocityL);
    gearboxR(direction, velocityR);

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

void moveInches(double inchesToTurn, double motorVelocity, bool forward){ //Move Drive a set amount of inches.

    degreesToTurn = inchesToDegrees(inchesToTurn);

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

    brakeDrive(coast);

}

double timeElapsed(timeUnits units){

    return myTimer.time(units);

}

void resetTimer(){
    
    myTimer.reset();

}

double getHeading(){
                    
    return InertialSensor.heading();

}

double getRoll(){
        
    return InertialSensor.roll();

}

void resetHeading(){
        
    InertialSensor.resetHeading();

}

void pregameCalibrate(){

    InertialSensor.calibrate();
    

    while (InertialSensor.isCalibrating()) {

        this_thread::sleep_for(100);

    }

}

void autoDrive(){
    
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

        move(fwd, leftAxismV, rightAxismV);

    }

    else{

        brakeDrive(brake);

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

void moveArm(){

    armAxisPCT = Controller1.Axis2.position(percent);
    armAxismV = armAxisPCT * percentToMVolts;

    if (abs(armAxisPCT) > 5){

        kickerArm.spin(fwd, armAxismV, voltageUnits::mV);

    }

    else{

        kickerArm.stop(brake);

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
