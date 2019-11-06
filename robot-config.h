#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"
//
vex::brain Brain;
vex::controller Controller = vex::controller();
vex::motor LeftF = vex::motor(vex::PORT1); //left front motor connected to port1 of vex brain
vex::motor RightF = vex::motor(vex::PORT2); //right front motor connected to port2
vex::motor LeftB = vex::motor(vex::PORT3); // left back motor connected to port 3
vex::motor RightB = vex::motor(vex::PORT4); //right back motor connected to port 4
vex::motor lift = vex::motor(vex::PORT5); //lift motor with pid connected to port5
vex::motor intake = vex::motor(vex::PORT6); //intake motor 1 connected to port 6
vex::motor intake2 = vex::motor(vex::PORT7); //intake motor 2 connected to port 7.
