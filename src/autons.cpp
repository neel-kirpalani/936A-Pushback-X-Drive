#include "vex.h"


// ---------------- Distance sensor helpers ----------------
// store object distance in variable so it can be used when needed (left distance sensor)
double getLeftWallDist() {
  return leftDist.objectDistance(inches);
}

// store object distance in variable so it can be used when needed (right distance sensor)
double getRightWallDist() {
  return rightDist.objectDistance(inches);
}

// ---------------- Wall alignment using holonomic strafing ----------------
// desired_dist_in: desired distance (inches) from the wall (average of left/right).
// tol_in: how close (inches) the average should be to desired_dist_in before we consider alignment "good enough".
// useLeftSensor:   true  -> use leftDist only (for left wall situations).
//                  false -> use rightDist only (for right wall situations).
void align_to_wall(double desired_dist_in, bool useLeftSensor, double tol_in) {
  int maxLoops = 40; // avoid inf loops
  
  for (int i = 0; i < maxLoops; i++) {
    // Read ONLY the chosen side sensor
    double dist = useLeftSensor ? getLeftWallDist() : getRightWallDist();

    // check: ignore obviously bad readings
    if (dist <= 0 || dist > 120) break;

    // positive means we are too far from the wall
    double error = dist - desired_dist_in;
    
    // if within tolerance, we are aligned; stop strafing
    if (fabs(error) < tol_in) {
      break;
    }

    double dir;
    if (useLeftSensor) {
      // Aligning to LEFT wall
      dir = (error > 0) ? -1.0 : 1.0;  // too far -> X-, too close -> X+
    } else {
      // Aligning to RIGHT wall
      dir = (error > 0) ? 1.0 : -1.0;  // too far -> X+, too close -> X-
    }
    
    // How much to move per correction step (in inches).
    // Start with 1.0 and tune smaller/larger on the real field.
    double stepX = dir * 1.0;

    // Current pose from odom
    double curX = chassis.get_X_position();
    double curY = chassis.get_Y_position();
    double curH = chassis.get_absolute_heading();

    // Target pose: adjust X only; keep Y and heading constant
    double targetX = curX + stepX;

    // Holonomic move: pure strafe in X while holding Y and heading
    chassis.holonomic_drive_to_pose(targetX, curY, curH);

    vex::wait(20, vex::msec);
  }
}


// Align and reset odom at a LEFT-side long-goal station.
void align_and_reset_long_goal_left() {
  double desired_dist_from_left_wall = 15.1; // inches, tune on field
  double tol = 0.5;

  // Use ONLY the left distance sensor because we are by the left wall.
  align_to_wall(desired_dist_from_left_wall, /*useLeftSensor=*/true, tol);

  // After alignment, reset odom to the known field pose for this station.
  // TODO: measure and fill these based on your field layout.
  double knownX = 60.0;
  double knownY = 24.0;
  double knownH = 0.0;

  chassis.set_coordinates(knownX, knownY, knownH);
}

// Align and reset odom at a RIGHT-side long-goal station.
void align_and_reset_long_goal_right() {
  double desired_dist_from_right_wall = 10.0; // inches, tune on field
  double tol = 0.5;

  // Use ONLY the right distance sensor because we are by the right wall.
  align_to_wall(desired_dist_from_right_wall, /*useLeftSensor=*/false, tol);

  // TODO: measure and fill for the right-side station.
  double knownX = -60.0;
  double knownY = 24.0;
  double knownH = 180.0;

  chassis.set_coordinates(knownX, knownY, knownH);
}


/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */

void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(12, 3.6, 0, 0.16, 0); // lateral PID constants

  chassis.set_heading_constants(6, .4, 0, 1, 0);  // used for drive-to-point and heading maintaining


  chassis.set_turn_constants(12, 0.15, 0, 0.13, 15); // angular PID constants (MaxVoltage, kP, kI, kD, startI).

  chassis.set_swing_constants(12, .3, .001, 2, 15); // used for swing turns (one side still, one side moves)

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.5, 300, 4000); // desides when it is "close enough" to the target and stops
  chassis.set_turn_exit_conditions(1, 300, 3000); // same concept as drive exit conditions but for turns 
  chassis.set_swing_exit_conditions(1, 300, 3000); // same but for swing turns
}

/**
 * Sets constants to be more effective for odom movements.
 * For functions like drive_to_point(), it's often better to have
 * a slower max_voltage and greater settle_error than you would otherwise.
 */

void odom_constants(){
  default_constants();
  chassis.heading_max_voltage = 10;
  chassis.drive_max_voltage = 12; // change back to default value of 8 if any issues occur
  chassis.drive_settle_error = 0.5;
  chassis.boomerang_lead = 0; // change back to default value of 0.5 if any issues occur
  chassis.drive_min_voltage = 0;
  chassis.drive_timeout = 3000;
}

/**
 * The expected behavior is to return to the start position.
 */

void drive_test(){
  chassis.drive_distance(6);
  chassis.drive_distance(12);
  chassis.drive_distance(18);
  chassis.drive_distance(-36);
}

/**
 * The expected behavior is to return to the start angle, after making a complete turn.
 */

void turn_test(){
  chassis.turn_to_angle(5);
  chassis.turn_to_angle(30);
  chassis.turn_to_angle(90);
  chassis.turn_to_angle(180);
  chassis.turn_to_angle(225);
  chassis.turn_to_angle(0);

  
}

/**
 * Should swing in a fun S shape.
 */

void swing_test(){
  chassis.left_swing_to_angle(90);
  chassis.right_swing_to_angle(0);
}

/**
 * A little of this, a little of that; it should end roughly where it started.
 */

void full_test(){
  chassis.drive_distance(24);
  chassis.turn_to_angle(-45);
  chassis.drive_distance(-36);
  chassis.right_swing_to_angle(-90);
  chassis.drive_distance(24);
  chassis.turn_to_angle(0);
}

/**
 * Doesn't drive the robot, but just prints coordinates to the Brain screen 
 * so you can check if they are accurate to life. Push the robot around and
 * see if the coordinates increase like you'd expect.
 */

void odom_test(){
  chassis.set_coordinates(0, 0, 0);
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5,20, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(5,40, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(5,60, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5,80, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(5,100, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    wait(100, msec);
  }
}

/**
 * Should end in the same place it began, but the second movement
 * will be curved while the first is straight.
 */

void tank_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.turn_to_point(24, 24);
  chassis.drive_to_point(24,24);
  chassis.drive_to_point(0,0);
  chassis.turn_to_angle(0);
}

/**
 * Drives in a square while making a full turn in the process. Should
 * end where it started.
 */

void holonomic_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.holonomic_drive_to_pose(0, 18, 90);
  chassis.holonomic_drive_to_pose(18, 0, 180);
  chassis.holonomic_drive_to_pose(0, 18, 270);
  chassis.holonomic_drive_to_pose(0, 0, 0);
}

void left_side_routine(){
  // add code in here for auton 1
  

  chassis.set_coordinates(0,0,0);
  //stage1.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
  //intakeMotor1.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
  //intakeMotor2.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
  chassis.holonomic_drive_to_pose(29,-2,0);
  //wait(300, msec);
  //solenoidA.set(true);
  wait(150, msec);
  chassis.holonomic_drive_to_pose(29, 4, 0);
  wait(100, msec);
  DriveL.stop(vex::brakeType::brake);
  DriveR.stop(vex::brakeType::brake);
  solenoidC.set(true);
  stage1.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
  intakeMotor1.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
  intakeMotor2.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
  


  DriveL.stop(coast);
  DriveR.stop(coast); 
  wait(2000, msec);
  //intakeMotor1.stop(vex::brakeType::coast);
  //intakeMotor2.stop(vex::brakeType::coast);
  
  
  

  chassis.holonomic_drive_to_pose(29, -11.12, 0);
  wait(50, msec);
  chassis.turn_to_angle(180); // face the long goal
  wait(100, msec);
  stage1.stop(vex::brakeType::coast);
  solenoidC.set(false);
  solenoidA.set(true);
  wait(50, msec);
  chassis.holonomic_drive_to_pose(23.1, -39, 180); // drive to long goal
  wait(100, msec);
  DriveL.stop(vex::coast);
  DriveR.stop(vex::coast);
  wait(50, msec);
  solenoidB.set(true);
  wait(1500, msec);
  solenoidB.set(false);
  chassis.holonomic_drive_to_pose(24.2, -22, 180);
  solenoidA.set(false);
  wait(100, msec);
  chassis.turn_to_angle(225);
  wait(25, msec);
  //chassis.holonomic_drive_to_pose(7, -35.822, 225);
  //wait(200, msec);
  chassis.holonomic_drive_to_pose(-15.5, -50, 225);
  wait(100, msec);
  solenoidB.set(true);
  wait(1000, msec);
  solenoidB.set(false);
  chassis.holonomic_drive_to_pose(7.373, -36, 270);
  wait(50, msec);
  chassis.holonomic_drive_to_pose(-65, -17, 0);
  wait(200, msec);
  chassis.holonomic_drive_to_pose(-62, 5.5, 0);
  wait(250, msec);
  solenoidC.set(true);
  wait(50, msec);
  stage1.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
  wait(2000, msec);
  stage1.stop(coast);
  wait(25, msec);
  solenoidC.set(false);
  wait(100, msec);
  chassis.holonomic_drive_to_pose(-60.7, -11, 0);
  solenoidA.set(true);
  wait(100, msec);
  chassis.turn_to_angle(180);
  wait(100, msec);
  chassis.holonomic_drive_to_pose(-63.4, -34, 180);
  wait(250, msec);
  solenoidB.set(true);
  wait(2000, msec);
  solenoidB.set(false); 
  
}

void test_dist_alignment(){
  // add code in here for auton 2
  chassis.set_coordinates(0,0,0);
  wait(2000, msec);
  align_to_wall(15.1, true, 0.25);
  
}

void skills_auton() {
  odom_constants();
  chassis.set_coordinates(0,0,0);

}