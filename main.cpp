#include "robot-config.h"
#include "iostream"
vex::competition    Competition;
void pre_auton( void ) {
    
}
float C = 12.566;
float clickperinch = 360/C;
float distance = 23.42;
float first_path = clickperinch * distance;
float distance2 = 11.708;
float second_path = clickperinch * distance2;
float turndeg= C / 4;
float distanceTravelled = turndeg / 2.54;

float liftang = lift.rotation(vex::rotationUnits::deg);
float liftvelocity = 0;
float pvallift = 0.1;
float errorlift = 0;
bool liftmov = 0;
float liftvelopercentage=0.5;
float temps = 0;

void autonomous( void ) {
  vex::task forward;
  vex::task backward;
  vex::task turn;
  vex::task forward2;
  
}

void foward() {
  //robot picks up the cubes
    LeftF.spinFor(- 1 * first_path, vex::deg);
    RightF.spinFor(first_path, vex::deg);
    LeftB.spinFor(first_path, vex::deg);
    RightB.spinFor(- 1 * first_path, vex::deg);
    intake.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
    intake2.spin(vex::directionType::fwd,-100,vex::velocityUnits::pct);
    intake.stop();
    intake2.stop();
}

void backward() {
  //robot goes back
    LeftF.spinFor(-1 * first_path, vex::deg);
    RightF.spinFor(-1 * first_path, vex::deg);
    LeftB.spinFor(first_path, vex::deg);
    RightB.spinFor(first_path, vex::deg);
}

void turn() {
  //robot turns 90°
  LeftF.spinFor(-1 * distanceTravelled , vex::deg);
  RightF.spinFor(distanceTravelled, vex::deg);
  LeftB.spinFor(-1 * distanceTravelled, vex::deg);
  RightB.spinFor(distanceTravelled, vex::deg);
}

void forward2() {
  //robot goes toward goal
    LeftF.spinFor(- 1 * first_path, vex::deg);
    RightF.spinFor(first_path, vex::deg);
    LeftB.spinFor(first_path, vex::deg);
    RightB.spinFor(- 1 * first_path, vex::deg);
}

void stack() {
  /* code for liftang to stack
  *
  *
  */

  intake.spin(vex::directionType::fwd,-50,vex::velocityUnits::pct);
  intake2.spin(vex::directionType::fwd,50,vex::velocityUnits::pct);
  liftmov=true;
  lift.spin(vex::directionType::fwd, 85*liftvelopercentage, vex::velocityUnits::pct);
  liftang = lift.rotation(vex::rotationUnits::deg);
  LeftF.spin(vex::directionType::fwd,30,vex::velocityUnits::pct);
  RightF.spin(vex::directionType::fwd,-30,vex::velocityUnits::pct);
  LeftB.spin(vex::directionType::fwd,-30,vex::velocityUnits::pct);
  RightB.spin(vex::directionType::fwd,30,vex::velocityUnits::pct);
  vex::task::sleep(500);
}

void usercontrol( void ) {
    #include "robot-config.h"
    while(true){
        Brain.Screen.printAt(10,10,"LeftF %f",LeftF.rotation(vex::rotationUnits::deg));
        Brain.Screen.newLine();
        Brain.Screen.printAt(10,30,"RightF %f",RightF.rotation(vex::rotationUnits::deg));
        Brain.Screen.newLine();
        Brain.Screen.printAt(10,50,"LeftB %f",LeftB.rotation(vex::rotationUnits::deg));
        Brain.Screen.newLine();
        Brain.Screen.printAt(10,70,"RightB %f",RightB.rotation(vex::rotationUnits::deg));
        Brain.Screen.newLine();
        Brain.Screen.printAt(10,90,"Lift %f",lift.rotation(vex::rotationUnits::deg)+temps);
        Brain.Screen.newLine();
        if(Controller.Axis3.value()!=0||Controller.Axis1.value()!=0){
            LeftF.spin(vex::directionType::fwd,-Controller.Axis3.value()+(-Controller.Axis1.value()),vex::velocityUnits::pct);
            RightF.spin(vex::directionType::fwd,Controller.Axis3.value()+(-Controller.Axis1.value()),vex::velocityUnits::pct);
            LeftB.spin(vex::directionType::fwd,Controller.Axis3.value()+(Controller.Axis1.value()),vex::velocityUnits::pct);
            RightB.spin(vex::directionType::fwd,-Controller.Axis3.value()+(Controller.Axis1.value()),vex::velocityUnits::pct);
        }else{
            LeftF.stop();
            LeftB.stop();
            RightF.stop();
            RightB.stop();
        }
        if(Controller.ButtonR1.pressing()){
            intake.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
            intake2.spin(vex::directionType::fwd,-100,vex::velocityUnits::pct);
        }else if(Controller.ButtonR2.pressing()){
            intake.spin(vex::directionType::fwd,-100,vex::velocityUnits::pct);
            intake2.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
        }else{
            intake.stop(vex::brakeType::coast);
            intake2.stop(vex::brakeType::coast);
        }
        errorlift = (liftang-lift.rotation(vex::rotationUnits::deg));
        liftvelocity=errorlift*pvallift;
        
        if(liftvelocity>0.5|| liftvelocity<-0.5){
            lift.spin(vex::directionType::fwd,liftvelocity,vex::velocityUnits::pct);
        }else if(liftmov==false){
            lift.stop(vex::brakeType::brake);
        }
        if(Controller.ButtonY.pressing()){
            if(liftvelopercentage==1){
                while(Controller.ButtonY.pressing()){
                    vex::task::sleep(1);
                }
                liftvelopercentage = 0.25;
                Controller.rumble(".");
            }else{
                while(Controller.ButtonY.pressing()){
                    vex::task::sleep(1);
                }
                liftvelopercentage = 0.5;
                Controller.rumble("..");
            }
        }
        if(Controller.ButtonL1.pressing()){
            liftmov=true;
            lift.spin(vex::directionType::fwd,-50*liftvelopercentage,vex::velocityUnits::pct);
            liftang = lift.rotation(vex::rotationUnits::deg);
        }else if(Controller.ButtonL2.pressing()){
            liftmov=true;
            lift.spin(vex::directionType::fwd,50*liftvelopercentage,vex::velocityUnits::pct);
            liftang = lift.rotation(vex::rotationUnits::deg);
        }else{
            liftang=liftang+0;
            //lift.stop(vex::brakeType::brake);
        }
        if(Controller.ButtonUp.pressing()){
            liftmov=true;
            lift.spin(vex::directionType::fwd,-25*liftvelopercentage,vex::velocityUnits::pct);
            liftang = lift.rotation(vex::rotationUnits::deg);
        }else if(Controller.ButtonDown.pressing()){
            liftmov=true;
            lift.spin(vex::directionType::fwd,25*liftvelopercentage,vex::velocityUnits::pct);
            liftang = lift.rotation(vex::rotationUnits::deg);
        }else{
            liftang=liftang+0;
            //lift.stop(vex::brakeType::brake);
        }
        if(Controller.ButtonX.pressing()) {
            intake.spin(vex::directionType::fwd,-50,vex::velocityUnits::pct);
            intake2.spin(vex::directionType::fwd,50,vex::velocityUnits::pct);
            liftmov=true;
            lift.spin(vex::directionType::fwd, 85*liftvelopercentage, vex::velocityUnits::pct);
            liftang = lift.rotation(vex::rotationUnits::deg);
            LeftF.spin(vex::directionType::fwd,30,vex::velocityUnits::pct);
            RightF.spin(vex::directionType::fwd,-30,vex::velocityUnits::pct);
            LeftB.spin(vex::directionType::fwd,-30,vex::velocityUnits::pct);
            RightB.spin(vex::directionType::fwd,30,vex::velocityUnits::pct);
            vex::task::sleep(500);
            LeftF.stop();
            LeftB.stop();
            RightF.stop();
            RightB.stop();
            intake.stop();
            intake2.stop();
        }
        
        if(Controller.ButtonB.pressing()){
            LeftF.resetRotation();
            RightF.resetRotation();
            LeftB.resetRotation();
            RightB.resetRotation();
            temps=temps+lift.rotation(vex::rotationUnits::deg);
        }
    }
}
int main() {
    pre_auton();
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );                       
    while(1) {
      vex::task::sleep(100);
    }
}