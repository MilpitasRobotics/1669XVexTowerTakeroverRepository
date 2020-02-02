#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
controller Controller = controller();
motor LeftF = motor(PORT1); //left front motor connected to port1 of vex brain
motor RightF = motor(PORT2); //right front motor connected to port2
motor LeftB = motor(PORT3); // left back motor connected to port 3
motor RightB = motor(PORT4); //right back motor connected to port 4
motor lift = motor(PORT5); //lift motor with pid connected to port5
motor intake = motor(PORT6); //intake motor 1 connected to port 6
motor intake2 = motor(PORT7); //intake motor 2 connected to port 7.

void vexcodeInit(void) {
  // Nothing to initialize
}