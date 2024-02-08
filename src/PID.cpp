#include "vex.h"

PID::PID(double error, double kP, double kI, double kD, double startIntegral, double settleError, double settleTime, double timeout):
    error(error), 
    kP(kP), 
    kI(kI), 
    kD(kD), 
    startIntegral(startIntegral), 
    settleError(settleError), 
    settleTime(settleTime), 
    timeout(timeout)
{};

PID::PID(double error, double kP, double kI, double kD, double startIntegral):
    error(error), 
    kP(kP), 
    kI(kI), 
    kD(kD), 
    startIntegral(startIntegral)
{};

double PID::compute(double error){

    if(fabs(error) < startIntegral){ //StartIntegral is used to avoid "integral windup", the unexpected oscillating of speed when using this PID. Error is set to match the next phase of the accumulated error in order to stop this effect. Practically caps the integral
        accumulatedError += error;
    }

    if((error > 0 && previousError < 0)|| (error < 0 && previousError > 0)){ //Checks if error has crossed from negative to positive/positive to negative, if so, accumulatedError is reset and therefore the Integral output is 0
        accumulatedError = 0;
    }

    output = (kP * error) + (kI * accumulatedError) + (kD * (error - previousError)); //PID equation, the final movement output

    previousError = error;

    if(fabs(error) < settleError){
        timeSpentSettled += 10;
    }

    else{
        timeSpentSettled = 0;
    }

    timeSpentRunning += 10;

    if (output > 100){
        output = 100;
    }

    else if (output < -100){
        output = -100;
    }

    return (output);
}


void PID::setValues(double error = 0, double kP = 0, double kI = 0, double kD = 0, double startIntegral = 0, double settleError = 0, double settleTime = 0, double timeout = 0){

    //Possibly set values
    this-> error = error;
    this-> kP = kP;
    this-> kI = kI;
    this-> kD = kD;
    this-> startIntegral = startIntegral;
    this-> settleError = settleError;
    this-> settleTime = settleTime;
    this-> timeout = timeout;
    //Reset rest to 0
    this-> accumulatedError = 0;
    this-> previousError = 0;
    this-> output = 0;
    this-> timeSpentSettled = 0;
    this-> timeSpentRunning = 0;
    this-> degreesError = 0;
    this-> motorSpeed = 0;
    this-> moveEnabled = false;

    resetDegreePosition();
    resetHeading();
    
}


bool PID::isSettled(){

    if(timeSpentSettled > settleTime){
        return true;
    }

    else if(timeSpentRunning > timeout && timeout != 0){ // If timeout does equal 0, the move will never actually time out. Setting timeout to 0 is the same as setting it to infinity
        return true;
    }

    else{
        return false;
    }

}

void PID::moveFor(double inches){
    setValues(inches, 0, 0, 0, 2, 90, 250, 5000); //Please test and tune these values as required. Set P, I, and D values
    moveEnabled = true;
    while(moveEnabled){
        
        degreesError = ((inches * 360) / (wheelCircumference * gearRatio)) - calculateAverageMotorRotation();
        motorSpeed = compute(degreesError);
        move(fwd, motorSpeed, motorSpeed);

        if (isSettled()){
            moveEnabled = false;
            break;
        }

        vexDelay(deltaTime);
    }


}

void PID::turnFor(double degrees){
    setValues(degrees, 0, 0, 0, 5, 15, 250, 5000); //Please test and tune these values as required. Set P, I, and D values
    moveEnabled = true;
    while(moveEnabled){
        move(fwd, motorSpeed, -motorSpeed);

        if (isSettled()){
             degreesError = degrees - getHeading(); //Future note: try turning robot without resetting heading to see if it works
            motorSpeed = compute(degreesError);
            moveEnabled = false;
            break;
        }

        vexDelay(deltaTime);
    }
}