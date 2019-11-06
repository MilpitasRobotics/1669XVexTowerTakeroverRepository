#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"
//
vex::brain Brain;
vex::controller Controller = vex::controller();
vex::motor LeftF = vex::motor(vex::PORT1);
vex::motor RightF = vex::motor(vex::PORT2);
vex::motor LeftB = vex::motor(vex::PORT3);
vex::motor RightB = vex::motor(vex::PORT4);
vex::motor lift = vex::motor(vex::PORT5);
vex::motor intake = vex::motor(vex::PORT6);
vex::motor intake2 = vex::motor(vex::PORT7);
