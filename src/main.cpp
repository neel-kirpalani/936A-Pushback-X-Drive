#include "vex.h"

using namespace vex;
competition Competition;

// state variables
bool intakeAllRunning = false;
bool intakeAllForward = true;
bool solenoidAActive = false;
bool solenoidBActive = false;
bool solenoidCActive = false;


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
motor_group(LeftFront, LeftBack),

//Right Motors:
motor_group(RightFront, RightBack),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT17, // IMU port

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
PORT13, // vertical odom pod (rotational sensor port)

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2, 

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
-8,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
PORT12, // horizontal odom pod (rotational sensor port)

//Sideways tracker diameter (reverse to make the direction switch):
-2,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
4

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
        Brain.Screen.printAt(5, 140, "Left1");
        break;
      case 1:
        Brain.Screen.printAt(5, 140, "Odom Test");
        break;
      case 2:
        Brain.Screen.printAt(5, 140, "Right1");
        break;
      case 3:
        Brain.Screen.printAt(5, 140, "Test Distance Alignment");
        break;
      case 4:
        Brain.Screen.printAt(5, 140, "Drive");
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
      left1();
      break;
    case 1:         
      odom_test();
      break;
    case 2:
      right1();
      break;
    case 3:
      test_dist_alignment();
      break;
    case 4:
      drive_test();
      break;
    case 5:
      odom_test();
      break;
    case 6:
      odom_test();
      break;
    case 7:
      odom_test();
      break;
 }
}


// ---------------- Field-oriented holonomic drive ----------------
//
// Converts joystick inputs (field frame) into robot-frame forward/strafe
// using the current odom heading, then drives the X-drive manually.
//
// Assumptions:
//  - Axis3 = forward/back on field (up/down stick).
//  - Axis4 = strafe left/right on field (left/right stick).
//  - Axis1 = rotation (turn).
//  - chassis.odom.orientation_deg = robot heading in degrees, 0° = field "up".
//

// ---------------- Field-oriented holonomic drive (X-drive + JAR-Template) ----------------
//
// Goal:
//   - Axis3 up  = always "forward" on the FIELD.
//   - Axis3 down = always "backward" on the FIELD.
//   - Axis4 right = always "right" on the FIELD.
//   - Axis4 left  = always "left" on the FIELD.
//   - Axis1 = rotate robot in place (still robot-centric).
//
// Requirements:
//   - IMU/odom heading is 0° when the robot is pointed "upfield" at the start of driver.
//   - Drive motors are: LeftFront, LeftBack, RightFront, RightBack, on an X-drive.
//   - chassis.odom.orientation_deg is valid (JAR-template odom running).

void field_oriented_drive() {
  // 1) Read joystick as field-frame commands
  double f = Controller.Axis3.position(percent) / 100.0; // field forward (+) / back (-)
  double s = Controller.Axis4.position(percent) / 100.0; // field right (+) / left (-)
  double t = Controller.Axis1.position(percent) / 100.0; // robot-centric turn

  // 2) Deadband
  const double DEADBAND = 0.05;
  if (fabs(f) < DEADBAND) f = 0;
  if (fabs(s) < DEADBAND) s = 0;
  if (fabs(t) < DEADBAND) t = 0;

  // If nothing commanded, stop drive and return
  if (f == 0 && s == 0 && t == 0) {
    LeftFront.stop(brakeType::coast);
    LeftBack.stop(brakeType::coast);
    RightFront.stop(brakeType::coast);
    RightBack.stop(brakeType::coast);
    return;
  }

  // 3) Get robot heading (field-relative)
  // Make sure heading is zeroed at the start of driver control.
  double heading_deg = chassis.odom.orientation_deg;
  double h = heading_deg * M_PI / 180.0;  // radians

  // 4) Convert field-frame (f, s) into ROBOT-frame (forward, strafe)
  //
  // Robot-forward  =  f * cos(h) + s * sin(h)
  // Robot-strafe   = -f * sin(h) + s * cos(h)
  //
  // This makes "stick up" always push in the same field direction,
  // regardless of robot orientation.
  double robotFwd =  f * cos(h) + s * sin(h);
  double robotStr = -f * sin(h) + s * cos(h);

  // 5) X-drive mixing (robot frame)
  double lf = robotFwd + robotStr + t;
  double rf = robotFwd - robotStr - t;
  double lb = robotFwd - robotStr + t;
  double rb = robotFwd + robotStr - t;

  // 6) Normalize so max magnitude is 1.0
  double maxMag = fmax(fmax(fabs(lf), fabs(rf)), fmax(fabs(lb), fabs(rb)));
  if (maxMag > 1.0) {
    lf /= maxMag;
    rf /= maxMag;
    lb /= maxMag;
    rb /= maxMag;
  }

  // 7) Scale to motor percent
  const double MAX_PCT = 100.0;
  lf *= MAX_PCT;
  rf *= MAX_PCT;
  lb *= MAX_PCT;
  rb *= MAX_PCT;

  // 8) Send to motors
  LeftFront.spin(directionType::fwd, lf, velocityUnits::pct);
  LeftBack.spin(directionType::fwd, lb, velocityUnits::pct);
  RightFront.spin(directionType::fwd, rf, velocityUnits::pct);
  RightBack.spin(directionType::fwd, rb, velocityUnits::pct);
}




void usercontrol(void) {

  chassis.set_coordinates(chassis.odom.X_position, chassis.odom.Y_position, 0.0);

  while (1) {
    // --- Intake toggle (L1 and L2 control ALL intakes at once) ---
    static bool prevL1 = false, prevL2 = false;
    bool currL1 = Controller.ButtonL1.pressing();
    bool currL2 = Controller.ButtonL2.pressing();

    if (currL1 && !prevL1) { // L1 pressed (forward)
      if (!intakeAllRunning || !intakeAllForward) {
        intakeAllRunning = true;
        intakeAllForward = true;
      } else {
        intakeAllRunning = false;
      }
    }
    if (currL2 && !prevL2) { // L2 pressed (reverse)
      if (!intakeAllRunning || intakeAllForward) {
        intakeAllRunning = true;
        intakeAllForward = false;
      } else {
        intakeAllRunning = false;
      }
    }
    prevL1 = currL1;
    prevL2 = currL2;

    if (intakeAllRunning) {
      directionType dir = intakeAllForward ? vex::directionType::fwd : vex::directionType::rev;
      intakeMotor1.spin(dir, 100, vex::velocityUnits::pct);
      intakeMotor2.spin(dir, 100, vex::velocityUnits::pct);
      stage1.spin(dir, 100, vex::velocityUnits::pct);
    } else {
      intakeMotor1.stop(vex::brakeType::coast);
      intakeMotor2.stop(vex::brakeType::coast);
      stage1.stop(vex::brakeType::coast);
    }

    // --- Pneumatics toggle (R1, R2, X) ---
    static bool prevR1 = false, prevR2 = false, prevX = false;
    bool currR1 = Controller.ButtonR1.pressing();
    bool currR2 = Controller.ButtonR2.pressing();
    bool currX  = Controller.ButtonX.pressing();

    // Toggle Solenoid A (R1)
    if (currR1 && !prevR1) {
      solenoidAActive = !solenoidAActive;
    }
    prevR1 = currR1;

    // Toggle Solenoid B (R2)
    if (currR2 && !prevR2) {
      solenoidBActive = !solenoidBActive;
    }
    prevR2 = currR2;

    // Toggle Solenoid C (X)
    if (currX && !prevX) {
      solenoidCActive = !solenoidCActive;
    }
    prevX = currX;

    // Set pneumatic states
    solenoidA.set(solenoidAActive);
    solenoidB.set(solenoidBActive);
    solenoidC.set(solenoidCActive);

    // --- Chassis drive code (holonomic) ---
    chassis.control_holonomic();

    // --- Chassis drive code (holonomic field orientated) ---
    //field_oriented_drive();

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
