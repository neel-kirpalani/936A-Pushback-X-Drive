#pragma once
#include "JAR-Template/drive.h"

class Drive;

extern Drive chassis;

void default_constants();

void drive_test();
void turn_test();
void swing_test();
void full_test();
void odom_test();
void tank_odom_test();
void holonomic_odom_test();
void left_side_routine();
void test_dist_alignment();
void skills_auton();
void left1();
void right1();
// distance sensor variables
double getLeftWallDist();
double getRightWallDist();

// alignment algorithms 
void align_to_wall(double desired_dist_in, double tol_in = 0.5);
void align_and_reset_long_goal_left();
void align_and_reset_long_goal_right();

