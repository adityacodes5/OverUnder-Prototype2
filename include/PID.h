#pragma once

class PID{

    public:

        double error = 0;
        double kP = 0;
        double kI = 0;
        double kD = 0;
        double startIntegral = 0;
        double settleError = 0;
        double settleTime = 0;
        double timeout = 0;
        double accumulatedError = 0;
        double previousError = 0;
        double output = 0;
        double timeSpentSettled = 0;
        double timeSpentRunning = 0;

        PID(double error, double kP, double kI, double kD, double startIntegral, double settleError, double settleTime, double timeout);

        PID(double error, double kP, double kI, double kD, double startIntegral);

        double compute(double error);

        bool isSettled();

  


};
