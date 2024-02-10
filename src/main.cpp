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
PID pid(0, 0, 0, 0, 0);
// define your global instances of motors and other devices here


void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  pregameCalibrate();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}


void autonomous(void) {
  //closeSideAuton();
  //matchLoadAuton();
  move(fwd, 100, 100);
}



void usercontrol(void) {

  pregameCalibrate(); //REMOVE LATER
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
      int timee = 5000;
      //prog skills
      pid.moveFor(1, 300, timee, false, 0);// to start program put it upgainst the far side pole and it should be parelle to the NET
      pid.turnFor(-45);
      pid.moveFor(30, 300, timee, false, 1); // goes to net
      pid.moveFor(-5, 300, timee, false, 0); 
      pid.turnFor(90);
      pid.moveFor(-2, 300, timee, false, 0);// goes to shooting pos
      //kickerArm.spinTo(360, rotationUnits::deg, true);
      kick(100, false);
      brakeDrive(coast);
      vexDelay(10000); //shoot
      kicker.stop();
      //kickerArm.spinTo(0, rotationUnits::deg, true);
      pid.moveFor(2, 300, timee, false, 0); 
      pid.turnFor(-50);
      pid.moveFor(30, 500, 1500, false, 0);
      pid.turnFor(5);
      pid.moveFor(100, 500, 5000, false, 0);
      pid.turnFor(45);
      wings.set(true);
      pid.moveFor(30, 500, 1500, false, 1);
      wings.set(false);
      pid.moveFor(-10, 500, 1000, false, 0);
      pid.turnFor(90);
      pid.moveFor(40, 500, 1500, false, 0);
      pid.turnFor(-45);
      wings.set(true);
      pid.moveFor(35, 500, 1500, false, 1);
      
      
      // pid.moveFor(100, 300, timee, false, 0);// go through side bar
      // pid.turnFor(45);
      // wings.set(true);
      // pid.moveFor(40, 300, timee, false, 1); // go to net
      // pid.moveFor(-10, 300, timee, false, 0);// get away from net
      // wings.set(false);
      // pid.turnFor(90);
      // pid.moveFor(30, 300, timee, false, 0);// go to front of net
      // pid.turnFor(-90);
      // pid.moveFor(20, 300, timee, false, 0);// push tribals into net
      // pid.turnFor(-90);
      // pid.moveFor(30, 300, timee, false, 1);
      // pid.moveFor(-30, 300, timee, false, 0);
      // pid.turnFor(90);
      // pid.moveFor(20, 300, timee, false, 0); // go to the 2nd time pushing in triballs to front of net 
      // pid.turnFor(-90);
      // pid.moveFor(30, 300, timee, false, 0);
      // pid.moveFor(-30, 300, timee, false, 0);
      //maybe left side push????
    }

    if(Controller1.ButtonA.pressing()){
      //AWP 
      pid.moveFor(1, 300, 3000, true, 0);
      pid.turnFor(50);
      wings.set(true);
      pid.moveFor(22, 300, 3000, true, 0);
      pid.turnFor(-35);
      pid.moveFor(30, 300, 1000, false, 0);
      wings.set(false);
      pid.moveFor(-7, 250, 1000, false, 1); 
      pid.turnFor(-90);
      pid.moveFor(30, 500, 1500, false, 0);
      pid.turnFor(30);
      pid.moveFor(20, 500, 1500, false, 2);
      pid.turnFor(70);
      wings.set(true);
      pid.moveFor(18, 300, 2000, true, 2);
      wings.set(true);
      pid.turnFor(90);
      wings.set(true);
      pid.moveFor(20, 500, 1500, true, 1);
      wings.set(false);
      pid.moveFor(-20, 500, 1500, false, 0);
      pid.turnFor(90);
      pid.moveFor(30, 500, 2000, false, 0);
      pid.turnFor(90);
      wings.set(true);
      pid.moveFor(10, 500, 1500, true, 0);
      wings.set(true);

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
