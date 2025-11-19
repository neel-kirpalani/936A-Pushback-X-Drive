using namespace vex;

// brain + controller
extern brain Brain;
extern controller Controller;

// drivetrain motors
extern motor LeftFront;
extern motor RightFront;
extern motor LeftBackA;
extern motor LeftBackB;
extern motor RightBackA;
extern motor RightBackB;
// drivetrain motor groups
extern motor_group LeftBack;
extern motor_group RightBack;
extern motor_group DriveL;
extern motor_group DriveR;

// intake motors
extern motor intakeMotor1;
extern motor intakeMotor2;
// stage 1 intake motors: 
extern motor intakeMotor3;
extern motor intakeMotor4;
// combine stage 1 intake motors
extern motor_group stage1;


void  vexcodeInit( void );