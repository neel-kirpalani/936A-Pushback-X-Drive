using namespace vex;

// brain + controller
extern brain Brain;
extern controller Controller;

// drivetrain motors
extern motor LeftFront;
extern motor RightFront;
extern motor LeftBack;
extern motor RightBack;
// drivetrain motor groups
extern motor_group DriveL;
extern motor_group DriveR;
// pnuematics 
extern digital_out solenoidA;
extern digital_out solenoidB;
extern digital_out solenoidC;

// intake motors
extern motor intakeMotor1;
extern motor intakeMotor2;
// stage 1 intake motors: 
extern motor intakeMotor3;
extern motor intakeMotor4;
// combine stage 1 intake motors
extern motor_group stage1;

// distance sensors
extern distance leftDist;
extern distance rightDist;


void  vexcodeInit( void );