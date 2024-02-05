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



    return (output);
}

bool PID::isSettled(){

    if(timeSpentSettled > settleTime){
        return true;
    }

    else if(timeSpentRunning > timeout){
        return true;
    }

    else{
        return false;
    }

}