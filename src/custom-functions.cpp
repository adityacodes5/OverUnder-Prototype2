#include "vex.h"
//#include <algorithm>
//#include <cstdlib>
#include <cmath>  // Include the cmath library for the exponential function

template <typename T>
T customMin(T a, T b) {
    return (a < b) ? a : b;
}

template <typename T>
T customMax(T a, T b) {
    return (a > b) ? a : b;
}


//#include <iostream>
using namespace std;
PID pid(0,0,0,0,0);
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
double distanceError;
double distanceErrorToTurn;
double distanceErrorDerivative;
double baseVelocityRatio;
double errorVelocity;
double originalInput;
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

//move PID variables
double originalInches;
double settlingInches;
double distanceDegrees;
double inchesError;
double speed;
bool enableMove;


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

int maxSpeed = 100*120;


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

        fabs(GearboxRF.position(rotationUnits::deg)) +
        fabs(GearboxRB.position(rotationUnits::deg)) +

        fabs(GearboxLF.position(rotationUnits::deg)) +
        fabs(GearboxLB.position(rotationUnits::deg)) +

        fabs(RightPush.position(rotationUnits::deg)) +
        fabs(LeftPush.position(rotationUnits::deg))

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

        brakeDrive(coast);
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
    settlingDist = 0;
    originalInput = 0;

    originalInches = 0;
    settlingInches = 0;
    distanceDegrees = 0;
    inchesError = 0;
    speed = 0;
    enableMove = false;

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
    //kickerArm.spinTo(540, rotationUnits::deg, true);
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


    //move(fwd, 60, 72);
    //vexDelay(1500);
    //brakeDrive(coast);
    //wait(100, msec);
    //grabTriball(100*120, true);
    //wait(250,msec);
    //move(directionType::rev, 40, 40);
    //wait(500,msec);
    //grabTriball(0, true);
    //brakeDrive(coast);
    //wait(200,msec);
    //move(fwd,100,100);
    //wait(500,msec);
    //brakeDrive(coast); 

    //wings.set(true);
    //vexDelay(500);
    //newMoveInches(13, 2);
    //newTurn(-30, 1);
    //vexDelay(250);
    //wings.set(false);
    //move(fwd, 70, 60);
    //vexDelay(500);
    //grabTriball(100*120, true);
    //vexDelay(500);
    //brakeDrive(coast);
    //grabTriball(0, true);
}

void setWings(){
    if(wings.value() == 1){
        wings.set(false);
    }
    else{
        wings.set(true);
    }
}

void newTurn(double targetAngle, double speed){
    resetTurnVariables();
    resetHeading();

    originalInput = targetAngle;
    settlingDist = fabs(targetAngle/45);
    if(targetAngle < 0) { 
        targetAngle = 360 - fabs(targetAngle);
        gyroscope.setHeading(359.9, rotationUnits::deg); // 359.9 to avoid 0/360 glitch
        leftTurn = true;
    }

    enableTurn = true;
    

    while(enableTurn){

        error = targetAngle - getHeading();
        speedL = targetAngle/5 * speed;
        speedR = -targetAngle/5 * speed;

        if(leftTurn){

        speedL = -fabs(originalInput)/5;
        speedR = fabs(originalInput)/5;

            if (speedL > -5 && speedR < 5){
                speedL = -5;
                speedR = 5;
            }

            if(fabs(error) <= settlingDist*speed){
            brakeDrive(brake);
            enableTurn = false;
            break;
            }
        }

        else if(!leftTurn){
            if (speedL < 5 && speedR > -5){
                speedL = 5;
                speedR = -5;
            }
            
            if(error <= 2){
            brakeDrive(brake);
            enableTurn = false;
            break;
            }
        }
        
        move(fwd, speedL, speedR);

        

        vexDelay(5);
    }

    vexDelay(200);
    brakeDrive(coast);
    


}

void newMoveInches(int inchesToMove, int multiplierSpeed){
    resetDegreePosition(); //Set all degree measurements to 0
    resetTurnVariables();

    distanceDegrees = inchesToDegrees(inchesToMove);
    originalInches = inchesToMove;
    settlingInches = (distanceDegrees/10);



    enableMove = true;

    while(enableMove){

        inchesError = distanceDegrees - calculateAverageMotorRotation();
        speed = inchesError/7 * multiplierSpeed;

        move(fwd, speed, speed);


        if(inchesError <= 1*deltaTime){
            brakeDrive(brake);
            enableTurn = false;
            break;
        }

        vexDelay(deltaTime);
    

    }    
}

void AWPclose(){
    wings.set(true);
    pid.moveFor(33, 300, 1300, true, 0);
    wings.set(false);
    Intake.spin(forward, 100, percentUnits::pct);
    pid.turnFor(120, 200);
    move(reverse, 100, 100);
    vexDelay(500);// at the net
    brakeDrive(coast);
    wings.set(false);
    
    pid.moveFor(10, 250, 300, false, 1); 
    pid.turnFor(100, 200);
    pid.moveFor(20, 200, 1000, false, 0);
    pid.turnFor(-45, 200);
    wings.set(true);
    pid.moveFor(30, 200, 1000, true, 0);
    wings.set(true);
    pid.turnFor(45, 200);
    pid.moveFor(15, 200, 1000, true, 0);
    wings.set(true);

    pid.turnFor(90, 200);
    pid.moveFor(23, 200, 1000, false, 0);
    pid.turnFor(-90, 200);
    move(fwd, 3, 3);

}

void AWP(){ //FINAL
    pid.moveFor(-25, 300, 1300, true, 0);
    wings.set(true);
    pid.turnFor(-60, 200);
    grabTriball(100, true);;
    move(reverse, 100, 100);
    grabTriball(100, false);
    vexDelay(500);
    brakeDrive(coast);

    wings.set(false);
    pid.moveFor(10, 250, 500, false, 1);
    wings.set(false);
    pid.turnFor(90, 200);
    grabTriball(maxSpeed, false);

        wings.set(false);
    pid.arcTurn(70, 80, 300, 1200, false);// go to middle conner 
    pid.turnFor(65, 300);
        wings.set(true);

    pid.moveFor(27, 200, 1000, false, 0); //go to middle triball

        wings.set(true);
    pid.turnFor(90, 300);
    grabTriball(maxSpeed, false);
    pid.moveFor(30, 300, 700, false, 1);//go to net 
    pid.moveFor(-30, 300, 800, false, 0);
    wings.set(false);


    pid.turnFor(90, 200);//go to pole 
    pid.moveFor(40, 100, 1300, false, 0);
    pid.turnFor(90, 200);
    pid.moveFor(10, 700, 500, false, 0);
    move(fwd, 3, 3);
    wings.set(true);

}

// void AWP(){ //FINAL
//     pid.moveFor(-25, 300, 1300, true, 0);
//     wings.set(true);
//     pid.turnFor(-60, 200);
//     grabTriball(100, true);;
//     move(reverse, 100, 100);
//     grabTriball(100, false);
//     vexDelay(500);
//     brakeDrive(coast);

    // wings.set(false);
    // pid.moveFor(10, 250, 500, false, 1);
    // wings.set(false);
    // pid.turnFor(40, 200);
    // grabTriball(maxSpeed, false);

    //     wings.set(false);
    // pid.arcTurn(70, 80, 300, 1200, false);// go to middle conner 
    // pid.turnFor(65, 300);
    //     wings.set(true);

// }

// void PROG(){
//           int timee = 5000;
//       //prog skills
//       pid.moveFor(1, 300, timee, false, 0);// to start program put it upgainst the far side pole and it should be parelle to the NET
//       pid.turnFor(-45, 200);
//       pid.moveFor(30, 300, timee, false, 1); // goes to net
//       pid.moveFor(-5, 300, timee, false, 0); 
//       pid.turnFor(90, 200);
//       pid.moveFor(-2, 300, timee, false, 0);// goes to shooting pos
//       //kickerArm.spinTo(360, rotationUnits::deg, true);
//       kick(100, false);
//       brakeDrive(coast);
//       vexDelay(10000); //shoot
//       kicker.stop();
//       //kickerArm.spinTo(0, rotationUnits::deg, true);
//       pid.moveFor(2, 300, timee, false, 0); 
//       pid.turnFor(-50, 200);
//       pid.moveFor(30, 500, 1500, false, 0);
//       pid.turnFor(5, 200);
//       pid.moveFor(100, 500, 5000, false, 0);
//       pid.turnFor(45, 200);
//       wings.set(true);
//       pid.moveFor(30, 500, 1500, false, 1);
//       wings.set(false);
//       pid.moveFor(-10, 500, 1000, false, 0);
//       pid.turnFor(90, 200);
//       pid.moveFor(40, 500, 1500, false, 0);
//       pid.turnFor(-45, 200);
//       wings.set(true);
//       pid.moveFor(35, 500, 1500, false, 1);
      
      
//       // pid.moveFor(100, 300, timee, false, 0);// go through side bar
//       // pid.turnFor(45);
//       // wings.set(true);
//       // pid.moveFor(40, 300, timee, false, 1); // go to net
//       // pid.moveFor(-10, 300, timee, false, 0);// get away from net
//       // wings.set(false);
//       // pid.turnFor(90);
//       // pid.moveFor(30, 300, timee, false, 0);// go to front of net
//       // pid.turnFor(-90);
//       // pid.moveFor(20, 300, timee, false, 0);// push tribals into net
//       // pid.turnFor(-90);
//       // pid.moveFor(30, 300, timee, false, 1);
//       // pid.moveFor(-30, 300, timee, false, 0);
//       // pid.turnFor(90);
//       // pid.moveFor(20, 300, timee, false, 0); // go to the 2nd time pushing in triballs to front of net 
//       // pid.turnFor(-90);
//       // pid.moveFor(30, 300, timee, false, 0);
//       // pid.moveFor(-30, 300, timee, false, 0);
//       //maybe left side push????
// }

void PROG(){
    kick(100, false);
    vexDelay(35000);
    kicker.stop(coast);
    wings.set(true);
    pid.turnFor(-30, 200);
    wings.set(true);
    Intake.spin(reverse, 100, percentUnits::pct);
    pid.moveFor(60, 200, 1500, true, 0);
    wings.set(true);
    Intake.spin(reverse, 100, percentUnits::pct);
    pid.turnFor(60, 300);
    Intake.spin(reverse, 100, percentUnits::pct);
    wings.set(false);
    pid.moveFor(100, 200, 5000, true, 0);
    //Intake.spin(reverse, 100, percentUnits::pct);
    wings.set(true);
    pid.moveFor(-30, 200, 1000, true, 0);
    Intake.spin(reverse, 100, percentUnits::pct);
    wings.set(true);
    pid.turnFor(-30, 300);
    pid.moveFor(30, 200, 1000, true, 0);
    Intake.spin(reverse, 100, percentUnits::pct);
        wings.set(true);
    pid.moveFor(-30, 200, 1000, true, 0);
    //Intake.spin(reverse, 100, percentUnits::pct);
    //    wings.set(true);
    //pid.moveFor(20, 200, 1500, true, 0);
    //Intake.spin(reverse, 100, percentUnits::pct);
    //    wings.set(true);
    //pid.moveFor(-20, 200, 1500, true, 0);
    //Intake.spin(reverse, 100, percentUnits::pct);
    //    wings.set(true);

}

double motorValueR(){
    return (fabs(GearboxRF.position(rotationUnits::deg)) + fabs(GearboxRB.position(rotationUnits::deg)))/2;
}

double motorValueL(){
    return (fabs(GearboxLF.position(rotationUnits::deg)) + fabs(GearboxLB.position(rotationUnits::deg)))/2;
}