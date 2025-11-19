#include "vex.h"

using namespace vex;
competition Competition;

// state variables
bool intake1Running = false;
bool intake1Forward = true;

bool intake2Running = false;
bool intake2Forward = true;

bool stage1Running = false;
bool stage1Forward = true;

// chassis configuration
Drive chassis(

//Pick your drive setup from the list below:
//ZERO_TRACKER_NO_ODOM
//ZERO_TRACKER_ODOM
//TANK_ONE_FORWARD_ENCODER
//TANK_ONE_FORWARD_ROTATION
//TANK_ONE_SIDEWAYS_ENCODER
//TANK_ONE_SIDEWAYS_ROTATION
//TANK_TWO_ENCODER
//TANK_TWO_ROTATION
//HOLONOMIC_TWO_ENCODER
//HOLONOMIC_TWO_ROTATION
//
HOLONOMIC_TWO_ROTATION, // select holonomic drive with two rotational sensors (2 odom wheels)

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(LeftFront, LeftBackA, LeftBackB),

//Right Motors:
motor_group(RightFront, RightBackA, RightBackB),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT16, // IMU port

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

0.8, // gear ratio (input teeth / output teeth) must be in decimal

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT2,     -PORT15,

//LB:      //RB: 
PORT7,     -PORT21,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
PORT12, // vertical odom pod (rotational sensor port)

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2.75, 

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
-2,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
PORT13, // horizontal odom pod (rotational sensor port)

//Sideways tracker diameter (reverse to make the direction switch):
-2.75,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
5.5

);

int current_auton_selection = 0;
bool auto_started = false;

/**
 * Function before autonomous. It prints the current auton number on the screen
 * and tapping the screen cycles the selected auton by 1. Add anything else you
 * may need, like resetting pneumatic components. You can rename these autons to
 * be more descriptive, if you like.
 */

void pre_auton() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  default_constants();

  while(!auto_started){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "JAR Template v1.2.0");
    Brain.Screen.printAt(5, 40, "Battery Percentage:");
    Brain.Screen.printAt(5, 60, "%d", Brain.Battery.capacity());
    Brain.Screen.printAt(5, 80, "Chassis Heading Reading:");
    Brain.Screen.printAt(5, 100, "%f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5, 120, "Selected Auton:");
    switch(current_auton_selection){
      case 0:
        Brain.Screen.printAt(5, 140, "Auton 1");
        break;
      case 1:
        Brain.Screen.printAt(5, 140, "Auton 2");
        break;
      case 2:
        Brain.Screen.printAt(5, 140, "Auton 3");
        break;
      case 3:
        Brain.Screen.printAt(5, 140, "Auton 4");
        break;
      case 4:
        Brain.Screen.printAt(5, 140, "Auton 5");
        break;
      case 5:
        Brain.Screen.printAt(5, 140, "Auton 6");
        break;
      case 6:
        Brain.Screen.printAt(5, 140, "Auton 7");
        break;
      case 7:
        Brain.Screen.printAt(5, 140, "Auton 8");
        break;
    }
    if(Brain.Screen.pressing()){
      while(Brain.Screen.pressing()) {}
      current_auton_selection ++;
    } else if (current_auton_selection == 8){
      current_auton_selection = 0;
    }
    wait(10,msec);
  }
}

/**
 * Auton function, which runs the selected auton. Case 0 is the default,
 * and will run in the brain screen goes untouched during preauton. Replace
 * drive_test(), for example, with your own auton function you created in
 * autons.cpp and declared in autons.h.
 */

void autonomous(void) {
  auto_started = true;
  switch(current_auton_selection){ 
    case 0:
      drive_test();
      break;
    case 1:         
      drive_test();
      break;
    case 2:
      turn_test();
      break;
    case 3:
      swing_test();
      break;
    case 4:
      full_test();
      break;
    case 5:
      odom_test();
      break;
    case 6:
      tank_odom_test();
      break;
    case 7:
      holonomic_odom_test();
      break;
 }
}


void usercontrol(void) {
  while (1) {
    // ---- Intake 1 (L1/L2 toggle) ----
    static bool prevL1 = false, prevL2 = false;
    bool currL1 = Controller.ButtonL1.pressing();
    bool currL2 = Controller.ButtonL2.pressing();

    if (currL1 && !prevL1) { // L1 pressed (forward)
      if (!intake1Running || !intake1Forward) {
        intake1Running = true;
        intake1Forward = true;
      } else {
        intake1Running = false;
      }
    }
    if (currL2 && !prevL2) { // L2 pressed (reverse)
      if (!intake1Running || intake1Forward) {
        intake1Running = true;
        intake1Forward = false;
      } else {
        intake1Running = false;
      }
    }
    prevL1 = currL1;
    prevL2 = currL2;
    if (intake1Running) {
      intakeMotor1.spin(intake1Forward ? vex::directionType::fwd : vex::directionType::rev, 100, vex::velocityUnits::pct);
    } else {
      intakeMotor1.stop(vex::brakeType::coast);
    }

    // ---- Intake 2 (R1/R2 toggle) ----
    static bool prevR1 = false, prevR2 = false;
    bool currR1 = Controller.ButtonR1.pressing();
    bool currR2 = Controller.ButtonR2.pressing();

    if (currR1 && !prevR1) { // R1 pressed (forward)
      if (!intake2Running || !intake2Forward) {
        intake2Running = true;
        intake2Forward = true;
      } else {
        intake2Running = false;
      }
    }
    if (currR2 && !prevR2) { // R2 pressed (reverse)
      if (!intake2Running || intake2Forward) {
        intake2Running = true;
        intake2Forward = false;
      } else {
        intake2Running = false;
      }
    }
    prevR1 = currR1;
    prevR2 = currR2;
    if (intake2Running) {
      intakeMotor2.spin(intake2Forward ? vex::directionType::fwd : vex::directionType::rev, 100, vex::velocityUnits::pct);
    } else {
      intakeMotor2.stop(vex::brakeType::coast);
    }

    // ---- Stage 1 (UP/DOWN toggle) ----
    static bool prevUp = false, prevDown = false;
    bool currUp = Controller.ButtonUp.pressing();
    bool currDown = Controller.ButtonDown.pressing();

    if (currDown && !prevDown) { // Down pressed (forward)
      if (!stage1Running || !stage1Forward) {
        stage1Running = true;
        stage1Forward = true;
      } else {
        stage1Running = false;
      }
    }
    if (currUp && !prevUp) { // Up pressed (reverse)
      if (!stage1Running || stage1Forward) {
        stage1Running = true;
        stage1Forward = false;
      } else {
        stage1Running = false;
      }
    }
    prevUp = currUp;
    prevDown = currDown;
    if (stage1Running) {
      stage1.spin(stage1Forward ? vex::directionType::fwd : vex::directionType::rev, 100, vex::velocityUnits::pct);
    } else {
      stage1.stop(vex::brakeType::coast);
    }

    // --- Chassis drive code ---
    chassis.control_holonomic();

    wait(20, msec); // Prevent wasted resources
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
