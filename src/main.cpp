/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Author:       Aditya Makhija                                            */
/*    Created:      Tue Dec 26 2023                                           */
/*    Project:      Iteration 2 V1.0                                          */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here


void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  //pregameCalibrate();




  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}


void autonomous(void) {


  //closeSideAuton();
  //matchLoadAuton();

  startTurn(70, 90, 0, 0, 0);

  

}



void usercontrol(void) {

  int maxSpeed = 100*120;
  bool alertOnce30 = true;
  bool alertOnce15 = true;
  double leftAxisPCT;
  double rightAxisPCT;
  double leftAxismV;
  double rightAxismV;
  double test;
  bool driving;
  bool wingsTriggered;

  resetTimer();

  while (1) {
    //leftAxisPCT = Controller1.Axis3.position(percent) + Controller1.Axis4.position(percent);
    //rightAxisPCT = Controller1.Axis3.position(percent) - Controller1.Axis4.position(percent);
//
    //leftAxismV = leftAxisPCT*120;
    //rightAxismV = rightAxisPCT*120;
    //test = Controller1.Axis2.position(percent);

    //kickerArm.spin(fwd, test, percentUnits::pct);

    //GearboxLF.spin(fwd, test, percentUnits::pct);
    //GearboxLB.spin(fwd, leftAxismV, voltageUnits::mV);
    //GearboxRF.spin(fwd, rightAxismV, voltageUnits::mV);
    //GearboxRB.spin(fwd, rightAxismV, voltageUnits::mV);

    //RightPush.spin(fwd, rightAxismV, voltageUnits::mV);
   //LeftPush.spin(fwd, leftAxismV, voltageUnits::mV);

    moveArm();
    driving = autoDrive();

//    if(Controller1.ButtonX.pressing()){
//
//      brakeDrive(hold);
//
//    }
//    
//    else if(!Controller1.ButtonX.pressing()){
//
//      brakeDrive(coast);
//
//    }

    if (timeElapsed(seconds) >= 75 && alertOnce30){

      alertRumble(false);
      alertOnce30 = false;

    }

    if (timeElapsed(seconds) >= 90 && alertOnce15){

      alertRumble(true);
      alertOnce15 = false;

    }

    if (Controller1.ButtonL1.pressing()){

      grabTriball(maxSpeed, false);

    }

    else if (Controller1.ButtonL2.pressing()){

      grabTriball(maxSpeed, true);

    }

    else if (!Controller1.ButtonL1.pressing() && !Controller1.ButtonL2.pressing()){
        
      grabTriball(0, true);
      
    }

    if (Controller1.ButtonR2.pressing()){

      kick(100, true);

    }

    if (Controller1.ButtonA.pressing()){
      oldMoveInches(20, 100);       
    }

    if (Controller1.ButtonB.pressing()){
      moveStraight(50, 50, 0.01, 0, 0, 1); // Example: Move straight for 100 units with a velocity of 50
    }

    else if (!Controller1.ButtonR2.pressing()){

      kick(0, true);

    }
    
    if(Controller1.ButtonR1.pressing()){

      wingsTriggered = true;

    }

    if(wingsTriggered && Controller1.ButtonR1.pressing() == false){
      if(wings.value() == false){
        wings.set(true);
        wingsTriggered = false;
      }

      else if(wings.value() == true){
        wings.set(false);
        wingsTriggered = false;
      }
    }




    wait(5, msec);
  }
}

//
// Main will set up the competition functions and callbacks.
//
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
