#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// defining brain + controller
brain  Brain;
controller Controller;

// defining drivetrain motors
motor LeftFront = motor(PORT2, ratio6_1, true);
motor RightFront = motor(PORT15, ratio6_1, false);
motor LeftBackA = motor(PORT7, ratio6_1, true);
motor LeftBackB = motor(PORT14, ratio6_1, false);
motor RightBackA = motor(PORT21, ratio6_1, false);
motor RightBackB = motor(PORT10, ratio6_1, false);
// drivetrain motor groups
motor_group LeftBack(LeftBackA, LeftBackB);
motor_group RightBack(RightBackA, RightBackB);
motor_group DriveL(LeftFront, LeftBackA, LeftBackB);
motor_group DriveR(RightFront, RightBackA, RightBackB);
// define pneumatics ports
vex::digital_out solenoidA(Brain.ThreeWirePort.A);
vex::digital_out solenoidB(Brain.ThreeWirePort.B);


// define intake motor 1
motor intakeMotor1 = motor(PORT9, ratio6_1, true);
// define intake motor 2
motor intakeMotor2 = motor(PORT4, ratio6_1, false);
// define intake motors for stage 1:
motor intakeMotor3 = motor(PORT1, ratio6_1, true);
motor intakeMotor4 = motor(PORT11, ratio6_1, false);
// combine stage 1 intake motors into a motor group
motor_group stage1(intakeMotor3, intakeMotor4);




//The motor constructor takes motors as (port, ratio, reversed), so for example
//motor LeftFront = motor(PORT1, ratio6_1, false);

//Add your devices below, and don't forget to do the same in robot-config.h:


void vexcodeInit( void ) {
  // nothing to initialize
}