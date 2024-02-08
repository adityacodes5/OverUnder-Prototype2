import matplotlib.pyplot as plt
import math
import time
import numpy as np


class drive:
    # Drive Variables
    wheelCircumference = 4 * math.pi
    gearRatio = 1.5
    percentToMVolts = 12
    deltaTime = 10
    maxVoltage = 12000
    delayTime = 10

    error = 0
    kP = 0
    kI = 0
    kD = 0
    startIntegral = 0
    settleError = 0
    settleTime = 0
    timeout = 0
    accumulatedError = 0
    previousError = 0
    output = 0
    timeSpentSettled = 0
    timeSpentRunning = 0
    degreesError = 0
    motorSpeed = 0
    originalDistance = 0
    moveEnabled = False

    def setVariables():
        drive.error = 0
        drive.kP = 0
        drive.kI = 0
        drive.kD = 0
        drive.startIntegral = 0
        drive.settleError = 0
        drive.settleTime = 0
        drive.timeout = 0
        drive.accumulatedError = 0
        drive.previousError = 0
        drive.output = 0
        drive.timeSpentSettled = 0
        drive.timeSpentRunning = 0
        drive.degreesError = 0
        drive.motorSpeed = 0
        drive.originalDistance = 0
        drive.moveEnabled = False

class simulation:
    motorSpeed = 0
    output = 0
    trueMotorSpeed = 0
    motorDegrees = 0
    errorInches = 0
    noise = 0

    xVal1 = []
    yVal1 = []
    xVal2 = []
    yVal2 = []
    xVal3 = []
    yVal3 = []

    def compute(error):
        
        if(abs(error) < drive.startIntegral):
            drive.accumulatedError += error
        
        if((error > 0 and drive.previousError < 0) or (error < 0 and drive.previousError > 0)):
            drive.accumulatedError = 0

        output = (drive.kP*error) + (drive.kI*drive.accumulatedError) + (drive.kD*(error - drive.previousError))

        drive.previousError = error

        if(abs(error) < drive.settleError):
            drive.timeSpentSettled += drive.deltaTime

        else:
            drive.timeSpentSettled = 0
        
        drive.timeSpentRunning += drive.deltaTime

        if (output > 100):
            output = 100

        elif (output < -100):
            output = -100

        return output
    
    def isSettled():

        if(drive.timeSpentSettled > drive.settleTime):
            return True
        
        elif (drive.timeSpentRunning > drive.timeout and drive.timeout != 0):
            return True

        else:
            return False
        
    def graph():
        plt.plot(simulation.xVal1, simulation.yVal1, label = "Error(in)")
        plt.plot(simulation.xVal2, simulation.yVal2, label = "MotorSpeed(pct)")
        plt.plot(simulation.xVal3, simulation.yVal3, label = "Distance(in)")
        plt.xlabel('Time (ms)')
        plt.ylabel('MotorSpeed (pct)/Error (in)')
        plt.title('Values vs Time')
        plt.legend()
        plt.show()

    def arrayAppend():
        simulation.xVal1.append(drive.timeSpentRunning)
        simulation.yVal1.append(simulation.errorInches)
        simulation.xVal2.append(drive.timeSpentRunning)
        simulation.yVal2.append(drive.motorSpeed)
        simulation.xVal3.append(drive.timeSpentRunning)
        simulation.yVal3.append(drive.originalDistance - simulation.errorInches)

        
    def moveFor(inches):

        drive.setVariables()
        drive.error = 0
        drive.kP = 0.5
        drive.kI = 0.01
        drive.kD = 0
        drive.startIntegral = 1
        drive.settleError = 5
        drive.settleTime = 500
        drive.timeout = 15000
        drive.originalDistance = inches

        drive.moveEnabled = True
        while(drive.moveEnabled):
            

            drive.degreesError = (inches * 360) / (drive.wheelCircumference*drive.gearRatio) - simulation.motorDegrees - simulation.noise
            drive.motorSpeed = simulation.compute(drive.degreesError)


            simulation.motorDegrees = simulation.motorDegrees + (drive.motorSpeed/10)
            simulation.errorInches = drive.degreesError/(drive.wheelCircumference*drive.gearRatio)
            print(drive.motorSpeed)
            print(" - ", simulation.errorInches)
            simulation.arrayAppend()

            simulation.noise = int(np.random.normal(abs(10*drive.motorSpeed/100), abs(10*drive.motorSpeed/100), 1).astype(int))

            if(simulation.isSettled()):
                simulation.graph()
                drive.moveEnabled = False
                break

            time.sleep(drive.deltaTime/10000)
        


            


    


        

simulation.moveFor(100)


