#pragma once

class PID{

    public:

        const double wheelCircumference = 4 * M_PI; //Circumference of the wheel
        const double gearRatio = 1.5; //Gear ratio of the drive
        const int percentToMVolts = 120;
        const int deltaTime = 10; //Time between each loop of the PID loop
        const int maxVoltage = 12000; //Max voltage of the motor
        const int delayTime = 10;

        //User adjustable variables
        double error = 0;
        double kP = 0;
        double kI = 0;
        double kD = 0;
        double startIntegral = 0;
        double settleError = 0;
        double settleTime = 0;
        double timeout = 0;

        //Computer adjusted variables
        double accumulatedError = 0;
        double previousError = 0;
        double output = 0;
        double timeSpentSettled = 0;
        double timeSpentRunning = 0;
        double degreesError = 0;
        double motorSpeed = 0;

        bool moveEnabled = false;

        //For each variable added, check "setvalues" in case you need to reset

        PID(double error, double kP, double kI, double kD, double startIntegral, double settleError, double settleTime, double timeout);

        PID(double error, double kP, double kI, double kD, double startIntegral);

        double compute(double error);

        void setValues(double error, double kP, double kI, double kD, double startIntegral, double settleError, double settleTime, double timeout);

        bool isSettled();

        void moveFor(double Inches);

        void turnFor(double degrees);

  


};
