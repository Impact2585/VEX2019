using namespace vex;
brain Brain;
// Left side reversed. Usually right side is reversed,
// but our gearing reverses the rotation again, making
// the left side the one that needs to be reversed
motor LeftBackMotor = motor(PORT1, false);
motor LeftFrontMotor = motor(PORT2, false);
motor RightBackMotor = motor(PORT10, true);
motor RightFrontMotor = motor(PORT9, true);
motor ArmMotor = motor(PORT11, true);
motor Feeder = motor(PORT20);
motor IntakeMotor = motor(PORT3);
motor ShooterMotor = motor(PORT7, false);
controller Controller1 = controller();
vex::vision::signature SIG_1 (1, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature BLUE_OBJ (2, -2979, 381, -1299, -1, 10427, 5213, 0.30000001192092896, 1);
vex::vision::signature RED_OBJ (3, 4131, 11843, 7987, -3099, -1531, -2315, 1, 1);
vex::vision::signature GREEN_FLAG (4, -3521, -2421, -2971, -7583, -6733, -7158, 1.7000000476837158, 1);
vex::vision::signature SIG_5 (5, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_6 (6, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::signature SIG_7 (7, 0, 0, 0, 0, 0, 0, 3, 0);
vex::vision::code BLUE_FLAG (BLUE_OBJ, GREEN_FLAG);
vex::vision::code RED_FLAG (RED_OBJ, GREEN_FLAG);
vex::vision VisionSensor (PORT5, 50, SIG_1, BLUE_OBJ, RED_OBJ, GREEN_FLAG, SIG_5, SIG_6, SIG_7);
const bool isProgrammingSkillsChallenge = false;
const bool NO_AUTON = false;
// Robot starting position
const bool isBlue = false;
const bool isFront = true;
