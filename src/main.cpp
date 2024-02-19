/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Author:       Aditya Makhija                                            */
/*    Created:      Tue Dec 26 2023                                           */
/*    Project:      Iteration 2 V1.0                                          */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;
using namespace std;


// A global instance of competition
competition Competition;
// define your global instances of motors and other devices here
bool isCalibrated;

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  pregameCalibrate();
  isCalibrated = true;
  Brain.Screen.print("It works!");
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}


void autonomous(void) {
  if(!isCalibrated){
    pregameCalibrate();
    isCalibrated = true;
  }
  //closeSideAuton();
  //matchLoadAuton();
  AWP();
  //AWPclose();
  //PROG();
}



void usercontrol(void) {
  PID pid(0,0,0,0,0);
  if(!isCalibrated){
    pregameCalibrate();
    isCalibrated = true;
  }
  int maxSpeed = 100*120;
  bool alertOnce30 = true;
  bool alertOnce15 = true;
  double leftAxisPCT;
  double rightAxisPCT;
  double leftAxismV;
  double rightAxismV;
  double test;
  bool driving;
  bool wingsTriggered = false;
  bool armTriggered = true;
  bool kickerTriggered;

  resetTimer();

  while (1) {
    moveArm();
    driving = autoDrive();

    if (timeElapsed(seconds) >= 75 && alertOnce30){

      alertRumble(false);
      alertOnce30 = false;

    }

    if (timeElapsed(seconds) >= 90 && alertOnce15){

      alertRumble(true);
      alertOnce15 = false;

    }

    if (Controller1.ButtonR1.pressing()){

      grabTriball(maxSpeed, false);

    }

    else if (Controller1.ButtonR2.pressing()){

      grabTriball(maxSpeed, true);

    }

    else if (!Controller1.ButtonR1.pressing() && !Controller1.ButtonR2.pressing()){
        
      grabTriball(0, true);
      
    }

    if (Controller1.ButtonL2.pressing()){

      kick(100, false);

    }

    else if (!Controller1.ButtonL2.pressing()){

      kick(0, true);

    }
    
    if(Controller1.ButtonL1.pressing()){

      wingsTriggered = true;

    }

    if(wingsTriggered && Controller1.ButtonL1.pressing() == false){
      if(wings.value() == false){
        wings.set(true);
        wingsTriggered = false;
      }

      else if(wings.value() == true){
        wings.set(false);
        wingsTriggered = false;
      }
    }

    if(Controller1.ButtonX.pressing()){

      armTriggered = true;

    }

    if(armTriggered && Controller1.ButtonX.pressing() == false){
      if(armHold.value() == false){
        armHold.set(true);
        armTriggered = false;
      }

      else if(armHold.value() == true){
        armHold.set(false);
        armTriggered = false;
      }
    }

    if(Controller1.ButtonB.pressing()){
      pid.arcTurn(45, 20, 300, 2000, false);
    }

    if(Controller1.ButtonA.pressing()){
      driveAndTurnPID(24, 0, 100, 1000, 200);
    }

    if(Controller1.ButtonY.pressing()){
      pid.turnFor(90, 200);//go to pole 
    }

    wait(5, msec);
  }
}

// Main will set up the competition functions and callbacks.
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
