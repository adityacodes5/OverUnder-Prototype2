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
    this-> leftDegrees = 0;
    this-> originalDegrees = 0;
    this-> moveEnabled = false;
    this-> leftTurn = false;
    this-> leftSpeed = 0;
    this-> rightSpeed = 0;
    this-> leftError = 0;
    this-> rightError = 0;

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

void PID::moveFor(double inches, double settleTime = 300, double timeout = 4000, bool openWingsHalfway = false, int outTakeAtEnd = false){
    setValues(inches, 0.2, 0.2, 2, 15, 20, settleTime, timeout); //Please test and tune these values as required. Set P, I, and D values
    moveEnabled = true;

    originalDegrees = ((inches * 360) / (wheelCircumference * gearRatio));

    while(moveEnabled){
        if(inches>=0){
            degreesError = ((inches * 360) / (wheelCircumference * gearRatio)) - calculateAverageMotorRotation();
        }

        if(inches<0){
            degreesError = ((inches * 360) / (wheelCircumference * gearRatio)) + calculateAverageMotorRotation();
        }
        
        if(openWingsHalfway){
            if((fabs(degreesError) < fabs(originalDegrees/2)) && wings.value() == false){
                wings.set(true);
            }
        }

        if(outTakeAtEnd == 1){
            if(timeSpentRunning < 500){
                Intake.spin(reverse, 100, percentUnits::pct);
            }
        }

        if(outTakeAtEnd == 2){
            if(timeSpentRunning < 500){
                Intake.spin(forward, 100, percentUnits::pct);
            }
        }
        motorSpeed = compute(degreesError);
        move(fwd, motorSpeed, motorSpeed);

        if (isSettled()){
            moveEnabled = false;

            // if(wings.value() == true){
            //     wings.set(false);
            // }

            if(outTakeAtEnd){
                Intake.stop();
            }

            break;
        }

        vexDelay(deltaTime);
    }


}

void PID::turnFor(double degrees, double settle){
    //Resets the gyroscope
    setValues(0, .5, 0, 0, 5, 5, settle, 2000); //Please test settle time
    if(degrees < 0) { 
        leftDegrees = 360 - fabs(degrees);
        gyroscope.setHeading(359, rotationUnits::deg); // 359.9 to avoid 0/360 glitch
        leftTurn = true;

    }
    if (degrees >= 0){
        gyroscope.setHeading(1, vex::rotationUnits::deg);
    }
    moveEnabled = true;
    while(moveEnabled){
        
        
        if(!leftTurn){
            degreesError = degrees - getHeading(); //Future note: try turning robot without resetting heading to see if it works
            motorSpeed = compute(degreesError);
            move(fwd, motorSpeed, -motorSpeed);
            
        }   
        else if(leftTurn){

            degreesError = getHeading() - fabs(leftDegrees);
            motorSpeed = compute(degreesError);
            move(reverse, motorSpeed, -motorSpeed);
        }

        if (isSettled()){
            
            moveEnabled = false;
            break;
        }

        vexDelay(deltaTime);
    }
}

void PID::arcTurn(double degrees, double radius, double settlingTime, double timeout, bool backwards){
    setValues(0, 1, 0.2, 15, 5, 5, settlingTime, timeout); //Please test settle time
    if(degrees < 0) { 
        leftDegrees = 360 - fabs(degrees);
        gyroscope.setHeading(359, rotationUnits::deg); // 359.9 to avoid 0/360 glitch
        leftTurn = true;

    }
    if (degrees >= 0){
        gyroscope.setHeading(1, vex::rotationUnits::deg);
    }
    moveEnabled = true;
    while(moveEnabled){
        
        
        if(!leftTurn){
            degreesError = degrees - getHeading(); //Future note: try turning robot without resetting heading to see if it works
            motorSpeed = compute(degreesError);
            leftSpeed = (motorSpeed*(radius+driveBase))/radius;
            Brain.Screen.printAt(10, 50, "Degrees Error: %f", degreesError);
            if(degrees < 0 || degrees > 350){
                gyroscope.setHeading(1, vex::rotationUnits::deg);
            }
            if(fabs(leftSpeed) >= 100){
                leftSpeed = 100;
                
            }
            rightSpeed = leftSpeed*((radius)/(radius+driveBase));
            if(!backwards){
                move(fwd, leftSpeed, rightSpeed);
            }
            else if(backwards){
                move(reverse, rightSpeed, leftSpeed);
            }
            
        }   
        else if(leftTurn){

            motorSpeed = compute(degreesError);
            rightSpeed = (motorSpeed*(radius+driveBase))/radius;
            Brain.Screen.printAt(10, 50, "Degrees Error: %f", degreesError);
            if(degrees < 0 || degrees > 350){
                gyroscope.setHeading(1, vex::rotationUnits::deg);
            }
            if(fabs(rightSpeed) >= 100){
                rightSpeed = 100;   
                
            }
            leftSpeed = rightSpeed*((radius)/(radius+driveBase));
            if(!backwards){
                move(fwd, leftSpeed, rightSpeed);
            }
            else if(backwards){
                move(reverse, rightSpeed, leftSpeed);
            }
        }

        

        if (isSettled()){
            
            moveEnabled = false;
            break;
        }

        vexDelay(deltaTime);
    }
}

//INCOMPLETE. DO NOT USE
void PID::arcTurnNew(double degrees, double radius, double settlingTime, double timeout, bool backwards){ //same as arcTurn but uses motor encoders instead of gyroscope
    setValues(0, 1, 0.2, 15, 5, 5, settlingTime, timeout); //Please test settle time
    if(degrees < 0) { 
        leftDegrees = 360 - fabs(degrees);
        gyroscope.setHeading(359, rotationUnits::deg); // 359.9 to avoid 0/360 glitch
        leftTurn = true;

    }
    if (degrees >= 0){
        gyroscope.setHeading(1, vex::rotationUnits::deg);
    }
    moveEnabled = true;
    while(moveEnabled){
        
        if(!leftTurn){
            leftSpeed = compute(leftError);
            rightSpeed = compute(rightError);
            move(fwd, leftSpeed, rightSpeed);
            
        }   
        else if(leftTurn){

        }

        

        if (isSettled()){
            
            moveEnabled = false;
            break;
        }

        vexDelay(deltaTime);
    }
}