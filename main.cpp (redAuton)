#include "iostream"
#include "vex.h"
vex::competition Competition;

void pre_auton(void) {}

/*
 * all this code is progressive and needs to be fixed and tuned
 */

// values for calculations for degree movement for auton
// clicks per inch and distance calculated to move certain degrees
/*float C = 12.566;
float clickperinch = 360/C;
//distance in inches
float distance = 23.42;
//first path: degrees robot needs to move forward to pick cubes
float first_path = clickperinch * distance;
float distance2 = 11.708;
//second path: dgrees robot moves to stacking place
float second_path = clickperinch * distance2;
//robot turns 90 degrees (or it is supposed to) to face the goal
float turndeg= C / 4;
float distanceTravelled = turndeg / 2.54;*/

// values needed for calculation for PID loop
float liftang = lift.rotation(vex::rotationUnits::deg);
float liftvelocity = 0;
float pvallift = 0.1;
float errorlift = 0;
bool liftmov = 0;
float liftvelopercentage = 1;
float temps = 0;
int targetval = -1950;
float wheel = 10.16;
float encPerCm = 360.0 / (wheel * M_PI);
float target = 0;
float targeterror = 0;
float liftanginitial = 0;
float macroliftvelo = 0;

// autonomous function
// robot moves forward to pick up cubes, goes back to original spot
// robot turns 90 degrees, goes forward, stacks, goes backward after stacking
// stacking is almost same code for macro in user control method
void goStraight(float distance, int time) {
  float totalEnc = distance*encPerCm; 
  lift.resetPosition();
  lift.resetRotation();
  LeftF.resetPosition();
  LeftF.resetRotation();
  LeftB.resetPosition();
  LeftB.resetRotation();
  RightB.resetPosition();
  RightB.resetRotation();
  RightF.resetPosition();
  RightF.resetRotation();
  
  LeftF.rotateTo(totalEnc, deg, 100.0, velocityUnits::pct, false);
  LeftB.rotateTo(totalEnc, deg, 100.0, velocityUnits::pct, false);
  RightF.rotateTo(-1*totalEnc, deg, 100.0, velocityUnits::pct, false);
  RightB.rotateTo(-1*totalEnc, deg, 100.0, velocityUnits::pct, false);

  intake.spin(vex::directionType::fwd, 100,vex::velocityUnits::pct);
  intake2.spin(vex::directionType::fwd,-100,vex::velocityUnits::pct);
  vex::task::sleep(time);
  intake.stop();
  intake2.stop(); 
  LeftF.stop();
  LeftB.stop();
  RightF.stop();
  RightB.stop();
}

void Forward(float distance, float speed) {
  float totalEnc = distance*encPerCm; 
  lift.resetPosition();
  lift.resetRotation();
  LeftF.resetPosition();
  LeftF.resetRotation();
  LeftB.resetPosition();
  LeftB.resetRotation();
  RightB.resetPosition();
  RightB.resetRotation();
  RightF.resetPosition();
  RightF.resetRotation();
  
  LeftF.rotateTo(totalEnc, deg, speed, velocityUnits::pct, false);
  LeftB.rotateTo(totalEnc, deg, speed, velocityUnits::pct, false);
  RightF.rotateTo(-1*totalEnc, deg, speed, velocityUnits::pct, false);
  RightB.rotateTo(-1*totalEnc, deg, speed, velocityUnits::pct, false);
  vex::task::sleep(1000);
  intake.stop();
  intake2.stop(); 
  LeftF.stop();
  LeftB.stop();
  RightF.stop();
  RightB.stop();
}

void towardsGoal(float distance) {
  float totalEnc = distance*encPerCm; 
  lift.resetPosition();
  lift.resetRotation();
  LeftF.resetPosition();
  LeftF.resetRotation();
  LeftB.resetPosition();
  LeftB.resetRotation();
  RightB.resetPosition();
  RightB.resetRotation();
  RightF.resetPosition();
  RightF.resetRotation();

  LeftF.rotateTo(totalEnc, deg, 40.0, velocityUnits::pct, false);
  LeftB.rotateTo(totalEnc, deg, 40.0, velocityUnits::pct, false);
  RightF.rotateTo(-1*totalEnc, deg, 40.0, velocityUnits::pct, false);
  RightB.rotateTo(-1*totalEnc, deg, 40.0, velocityUnits::pct, false);
  LeftF.stop();
  LeftB.stop();
  RightF.stop();
  RightB.stop();
}
float WHEEL_DIAMETER = 4.00;
float TURNING_DIAMETER = 13.00;
void turn(float degrees) {
  float turningRatio = TURNING_DIAMETER / WHEEL_DIAMETER;
  float wheel_degrees = turningRatio * degrees;
  LeftF.startRotateFor(wheel_degrees,vex::rotationUnits::deg);
  LeftB.startRotateFor(wheel_degrees,vex::rotationUnits::deg);
  RightF.startRotateFor(wheel_degrees,vex::rotationUnits::deg);
  RightB.startRotateFor(wheel_degrees,vex::rotationUnits::deg);
  vex::task::sleep(1000);
}
void stack() {
  intake.spin(vex::directionType::fwd, -10, vex::velocityUnits::pct);
  intake2.spin(vex::directionType::fwd, 10, vex::velocityUnits::pct);
  vex::task::sleep(100);
  wait(100, msec);
  lift.spin(vex::directionType::fwd, -80, vex::velocityUnits::pct);
  vex::task::sleep(1250);
  wait(1000, msec);
  Forward(2.54, 95.0);
  wait(500, msec);
  lift.spin(vex::directionType::fwd, 150,vex::velocityUnits::pct);
  intake.spin(vex::directionType::fwd, -45, vex::velocityUnits::pct);
  intake.spin(vex::directionType::fwd, 45, vex::velocityUnits::pct);
  LeftF.spin(vex::directionType::fwd,-40,vex::velocityUnits::pct);
  RightF.spin(vex::directionType::fwd,40,vex::velocityUnits::pct);
  LeftB.spin(vex::directionType::fwd,-40,vex::velocityUnits::pct);
  RightB.spin(vex::directionType::fwd,40,vex::velocityUnits::pct);
  vex::task::sleep(1000);
  LeftF.stop();
  LeftB.stop();
  RightF.stop();
  RightB.stop();
  lift.stop();
}
void autonomous( void ) {
  /*intake.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
  intake2.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  vex::task::sleep(1500);*/
  //wait(600, msec);
  goStraight(95.68, 2000);
  wait(400, msec);
  turn(-25);
  wait(400, msec);
  Forward(-80.9, 90.0);
  wait(400, msec);
  turn(20);
  wait(400, msec);
  goStraight(120.68, 2200);
  turn(131.89);
  wait(2000, msec);
  Forward(88.64, 100.0);
  LeftF.stop();
  RightF.stop();
  LeftB.stop();
  RightB.stop();
  stack();
  intake.stop();
  intake2.stop();
  while(true) {
    vex::task::sleep(100);
  }
}



void onePtAuton() {
  intake.spin(vex::directionType::fwd,-65,vex::velocityUnits::pct);
  intake2.spin(vex::directionType::fwd,65,vex::velocityUnits::pct);
  LeftF.spin(vex::directionType::fwd,50,vex::velocityUnits::pct);
  RightF.spin(vex::directionType::fwd,-50,vex::velocityUnits::pct);
  LeftB.spin(vex::directionType::fwd,-50,vex::velocityUnits::pct);
  RightB.spin(vex::directionType::fwd,50,vex::velocityUnits::pct);
  vex::task::sleep(2200);
}

void usercontrol(void) {
#include "robot-config.h"
  while (true) {
    Brain.Screen.printAt(10, 10, "LeftF %f",
                         LeftF.rotation(vex::rotationUnits::deg));
    Brain.Screen.newLine();
    Brain.Screen.printAt(10, 30, "RightF %f",
                         RightF.rotation(vex::rotationUnits::deg));
    Brain.Screen.newLine();
    Brain.Screen.printAt(10, 50, "LeftB %f",
                         LeftB.rotation(vex::rotationUnits::deg));
    Brain.Screen.newLine();
    Brain.Screen.printAt(10, 70, "RightB %f",
                         RightB.rotation(vex::rotationUnits::deg));
    Brain.Screen.newLine();
    Brain.Screen.printAt(10, 90, "Lift %f",
                         lift.rotation(vex::rotationUnits::deg) - temps);
    Brain.Screen.newLine();
    if (Controller.Axis3.value() != 0 || Controller.Axis1.value() != 0) {
      LeftF.spin(vex::directionType::fwd,
                 Controller.Axis3.value() + (Controller.Axis1.value()),
                 vex::velocityUnits::pct);
      RightF.spin(vex::directionType::fwd,
                  -Controller.Axis3.value() + (Controller.Axis1.value()),
                  vex::velocityUnits::pct);
      LeftB.spin(vex::directionType::fwd,
                 Controller.Axis3.value() + (Controller.Axis1.value()),
                 vex::velocityUnits::pct);
      RightB.spin(vex::directionType::fwd,
                  -Controller.Axis3.value() + (Controller.Axis1.value()),
                  vex::velocityUnits::pct);
    } else {
      LeftF.stop();
      LeftB.stop();
      RightF.stop();
      RightB.stop();
    }
    if (Controller.ButtonR1.pressing()) {
      intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
      intake2.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
    } else if (Controller.ButtonR2.pressing()) {
      intake.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
      intake2.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    } else {
      intake.stop(vex::brakeType::coast);
      intake2.stop(vex::brakeType::coast);
    }
    errorlift = (liftang - lift.rotation(vex::rotationUnits::deg));
    liftvelocity = errorlift * pvallift;
    /*
    if (liftvelocity > 0.5 || liftvelocity < -0.5) {
      lift.spin(vex::directionType::fwd, liftvelocity, vex::velocityUnits::pct);
    } else if (liftmov == false) {
      lift.stop(vex::brakeType::brake);
    }
    */
      
    if (Controller.ButtonL1.pressing()) {
      liftmov = true;
      lift.spin(vex::directionType::fwd, -100 * liftvelopercentage,
                vex::velocityUnits::pct);
      liftang = lift.rotation(vex::rotationUnits::deg);
    } else if (Controller.ButtonL2.pressing()) {
      liftmov = true;
      lift.spin(vex::directionType::fwd, 100 * liftvelopercentage,
                vex::velocityUnits::pct);
      liftang = lift.rotation(vex::rotationUnits::deg);
    }else if (Controller.ButtonUp.pressing()) {
      liftmov = true;
      lift.spin(vex::directionType::fwd, -25 * liftvelopercentage,
                vex::velocityUnits::pct);
      liftang = lift.rotation(vex::rotationUnits::deg);
    } else if (Controller.ButtonDown.pressing()) {
      liftmov = true;
      lift.spin(vex::directionType::fwd, 25 * liftvelopercentage,
                vex::velocityUnits::pct);
      liftang = lift.rotation(vex::rotationUnits::deg);
    } else {
      lift.stop(vex::brakeType::brake);
      //liftang = liftang + 0;
    }
    if (Controller.ButtonX.pressing()) {
      liftmov = true;
      liftanginitial = lift.rotation(vex::rotationUnits::deg);
      target = targetval;
      while(abs(float(target-lift.rotation(vex::rotationUnits::deg)))>100){
        targeterror = target-lift.rotation(vex::rotationUnits::deg);
        Brain.Screen.printAt(10, 10, "LeftF %f",
                            LeftF.rotation(vex::rotationUnits::deg));
        Brain.Screen.newLine();
        Brain.Screen.printAt(10, 30, "RightF %f",
                            RightF.rotation(vex::rotationUnits::deg));
        Brain.Screen.newLine();
        Brain.Screen.printAt(10, 50, "LeftB %f",
                            LeftB.rotation(vex::rotationUnits::deg));
        Brain.Screen.newLine();
        Brain.Screen.printAt(10, 70, "RightB %f",
                            RightB.rotation(vex::rotationUnits::deg));
        Brain.Screen.newLine();
        Brain.Screen.printAt(10, 90, "lift %f",
                            lift.rotation(vex::rotationUnits::deg) - temps);
        Brain.Screen.newLine();
        Brain.Screen.printAt(10, 110, "error %f",
                            targeterror)    ;
        Brain.Screen.newLine();
        Brain.Screen.printAt(10, 130, "velo %f",
                            macroliftvelo);
        Brain.Screen.newLine();
        if(targeterror>30){
          macroliftvelo = 0.3*targeterror;
        }else {
          macroliftvelo = 0.08*targeterror;
        } 
        lift.spin(vex::directionType::fwd, macroliftvelo,vex::velocityUnits::pct);

        if(abs(float(target-lift.rotation(vex::rotationUnits::deg)))<100) {
          lift.stop();
        }
      }
      lift.resetPosition();
      lift.resetRotation();
    }
    if (Controller.ButtonB.pressing()) {
      LeftF.resetRotation();
      RightF.resetRotation();
      LeftB.resetRotation();
      RightB.resetRotation();
      temps = lift.rotation(vex::rotationUnits::deg);
      
      while(Controller.ButtonB.pressing()){
        wait(1, msec);
      }
    }
  }
  
}
int main() {
  pre_auton();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  while (1) {
    vex::task::sleep(100);
  }
}
