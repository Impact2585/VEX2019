#include "robot-config.h"
/*---------------------------------------------------------------------------                                        
        Description: Team 2585's VEX Control Software for 2018-19          

Robot Configuration:
[Smart Port]   [Name]          [Type]            [Description]       [Reversed]
Motor Port 1   LeftBackMotor   V5 Smart Motor    Left side motor     false
Motor Port 2   LeftFrontMotor  V5 Smart Motor    Left side motor     false
Motor Port 10  RightBackMotor  V5 Smart Motor    Right side motor    true
Motor Port 9   RightFrontMotor V5 Smart Motor    Right side motor    true
Motor Port 11  ArmMotor        V5 Smart Motor    Arm motor           false
Motor Port 20  Feeder          V5 Smart Motor    Feeder              false
Motor Port 3   IntakeMotor     V5 Smart Motor    Intake motor        false
Motor Port 7   ShooterMotor    V5 Smart Motor    Shooter motor       false
Motor Port 5   VisionSensor    V5 Vision Sensor  Vision sensor       N/A

---------------------------------------------------------------------------*/

//Creates a competition object that allows access to Competition methods.
competition    Competition;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function. You must return from this function    */
/*  or the autonomous and usercontrol tasks will not be started. This        */
/*  function is only called once after the cortex has been powered on and    */ 
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton( void ) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  LeftBackMotor.resetRotation();
  LeftFrontMotor.resetRotation();
  RightBackMotor.resetRotation();
  RightFrontMotor.resetRotation();
}

// Robot measurements
const float wheelDiameter = 4.125; // inches
const float turningDiameter = 17.5; //inches (TODO: re-measure horizontally)
const float gearRatio = 1.0; // 1 turns of motor/turns of wheel
const float wheelCircumference = wheelDiameter * 3.141592653589;
const float inchesPerDegree = wheelCircumference / 360.0;
// should be turningDiameter/wheelDiameter
// Didn't work so 3.135 measured experimentally
const float encoderTicksPerDegree = 3.135;

void runIntake(float intakeSpeed){
    IntakeMotor.spin(directionType::fwd, intakeSpeed, velocityUnits::pct);
}

void runArm(float armSpeed) {
    if (armSpeed == 0) {
        ArmMotor.stop(brakeType::brake);
    } else {
        ArmMotor.spin(directionType::fwd, armSpeed, velocityUnits::pct);
    }
}

void runFeeder(float feederSpeed){
    if(feederSpeed == 0){
        Feeder.stop(brakeType::brake);
    } else {
        Feeder.spin(directionType::fwd, feederSpeed, velocityUnits::pct);
    }
}

void runDrive(float powerPCT, float rotationPCT) {
    //positive rotation --> turning right
    //negative rotation --> turning left
    LeftBackMotor.spin(directionType::fwd, powerPCT + rotationPCT, velocityUnits::pct);
    LeftFrontMotor.spin(directionType::fwd, powerPCT + rotationPCT, velocityUnits::pct);

    RightBackMotor.spin(directionType::fwd, powerPCT - rotationPCT, velocityUnits::pct);
    RightFrontMotor.spin(directionType::fwd, powerPCT - rotationPCT, velocityUnits::pct);
}

void powerDownShooter(float velocityPCT){
    ShooterMotor.setVelocity(velocityPCT - 0.5, velocityUnits::pct);
}

void powerUpShooter(float velocityPCT){
    ShooterMotor.setVelocity(velocityPCT + 8, velocityUnits::pct);
}

void autoDriveForward( float inches, float power ) { // distance in inches
    float degreesTurn = inches / inchesPerDegree * gearRatio;
    
    Controller1.Screen.print(inches);
    if(inches < 0){
        Brain.Screen.print("Backwards");
        degreesTurn *= -1;
        // TODO: run tests and hopefully remove
        LeftFrontMotor.setReversed(true);
        LeftBackMotor.setReversed(true);
        RightFrontMotor.setReversed(false);
        RightBackMotor.setReversed(false);
    }
    LeftFrontMotor.startRotateFor(degreesTurn, rotationUnits::deg, power, velocityUnits::pct);
    LeftBackMotor.startRotateFor(degreesTurn, rotationUnits::deg, power, velocityUnits::pct);
    RightFrontMotor.startRotateFor(degreesTurn, rotationUnits::deg, power, velocityUnits::pct);
    RightBackMotor.rotateFor(degreesTurn, rotationUnits::deg, power, velocityUnits::pct);

    LeftBackMotor.setReversed(false);
    LeftFrontMotor.setReversed(false);
    RightBackMotor.setReversed(true);
    RightFrontMotor.setReversed(true);
}    

void autoTurn( float degrees ) {
    // Note: +90 degrees is a right turn
    float encoderDegrees = encoderTicksPerDegree * degrees;
    LeftBackMotor.startRotateFor(encoderDegrees, rotationUnits::deg, 50, velocityUnits::pct);
    LeftFrontMotor.startRotateFor(encoderDegrees, rotationUnits::deg, 50, velocityUnits::pct);

    RightBackMotor.startRotateFor(-encoderDegrees, rotationUnits::deg, 50, velocityUnits::pct);
    RightFrontMotor.rotateFor(-encoderDegrees, rotationUnits::deg, 50, velocityUnits::pct);
}

void autoPowerUpShooter(float power) {
    int shooterVelocity = ShooterMotor.velocity(velocityUnits::pct);
    while (shooterVelocity < power - 10) {
        powerUpShooter(power);
        shooterVelocity = ShooterMotor.velocity(velocityUnits::pct);
        task::sleep(10);
    }
    ShooterMotor.spin(directionType::fwd, power, velocityUnits::pct);
}

void autoPowerDownShooter() {
    int shooterVelocity = ShooterMotor.velocity(velocityUnits::pct);
    while (shooterVelocity > 10) {
        powerDownShooter(0);
        shooterVelocity = ShooterMotor.velocity(velocityUnits::pct);
    }
    ShooterMotor.stop(brakeType::coast);
}

void autoShoot(int shootPower) {
    int intakePower = 100;
    int feederPower = 60;
    // Spin up the shooter
    autoPowerUpShooter(shootPower);
    task::sleep(500);
    // Run the intake and feeder to bring the ball up
    runIntake(intakePower);
    runFeeder(feederPower);
    task::sleep(2000);
    // At the same time, run the shooter to shoot the ball
    runIntake(0);
    runFeeder(0);
    autoPowerDownShooter();
}

void pointTo(vision::signature sig) {
    //camera image is 316 pixels wide, so the center is 316/2
    int screenMiddleX = 316 / 2;
    bool isLinedUp = false;
    while(!isLinedUp) {
        //snap a picture
        VisionSensor.takeSnapshot(sig);
        //did we see anything?
        if(VisionSensor.objectCount > 0) {
            //where was the largest thing?
            if(VisionSensor.largestObject.centerX < screenMiddleX - 5) {
                //on the left, turn left
                runDrive(0, -10);
            } else if (VisionSensor.largestObject.centerX > screenMiddleX + 5) {
                //on the right, turn right
                runDrive(0, 10);
            } else {
                //in the middle, we're done lining up
                isLinedUp = true;
                runDrive(0, 0);
            }
        } else {
            return;
        }
    }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control the robot during the autonomous phase of    */
/*  a VEX Competition.                                                       */
/*---------------------------------------------------------------------------*/

void autonomous( void ) {
    if (NO_AUTON) return;
    
    if(isFront){
        // Spin up while driving to flag
        ShooterMotor.spin(directionType::fwd, 100, velocityUnits::pct);
        autoDriveForward(24, 25);
        task::sleep(3000);

        // Run the intake and feeder to shoot
        IntakeMotor.startRotateFor(100, rotationUnits::rev, 100, velocityUnits::pct);
        Feeder.startRotateFor(10, rotationUnits::rev, 60, velocityUnits::pct);
        task::sleep(2000);
        IntakeMotor.stop(brakeType::coast);
        Feeder.stop(brakeType::coast);
        ShooterMotor.stop(brakeType::coast);
        
        // Drive forward to hit bottom flag
        autoDriveForward(24, 30);
        
        // Drive back to get in position for parking
        autoDriveForward(-24, 50);
        autoDriveForward(-50, 100);
        
        // Turn to platform
        if(isBlue){
            autoTurn(-90);
        } else { // red side
            autoTurn(90);
        }

        // Raise arm for parking
        ArmMotor.spin(directionType::rev, 20, velocityUnits::pct);
        task::sleep(500);
        ArmMotor.stop(brakeType::brake);
        
        // Drive on to platform to park
        autoDriveForward(-80, 100); // negative because the bot is backwards now

    } else { // in the back
        // Drive straight to intake ball beneath disk
        runIntake(127.0);        
        autoDriveForward( 3.3 * 12, 50.0 );
        IntakeMotor.rotateFor(2.0, vex::timeUnits::sec, 127.0, vex::velocityUnits::pct);
        runIntake(0.0);
        return;
    }
}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                           Programming Skills                              */
/*                                                                           */
/*  This task is used to control the robot during the autonomous phase of    */
/*  a VEX Competition. For use during programming skills event               */
/*---------------------------------------------------------------------------*/

void programmingSkills ( void ) {
    // TODO: fill out
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol( void ) {
    //Use these variables to set the speed of the arm, intake, and shooter.
    int armSpeedPCT = 20;
    int intakeSpeedPCT = 100;
    int feederSpeedPCT = 60;
    int currentShooterSpeedPCT = 0;
    
    bool isReversed = false;
    bool shouldShoot = false;
    Controller1.Screen.print("Welcome Aboard!");
    Controller1.Screen.newLine();
    Controller1.Screen.print("Get ready to rumble!!!");
    Controller1.Screen.newLine();

    Controller1.Screen.print("FORWARD MODE!");
    Controller1.Screen.newLine();
    Controller1.rumble("--..-");

    // Keep track of past button state to detect inital presses
    bool wasIntakePressed = false;
    bool wasUpPressed = false;
    bool wasDownPressed = false;
    
    while (1) {
        // This is the main execution loop for the user control program.
        // Each time through the loop your program should update motor + servo

        //Drive Control
        int powerPCT = Controller1.Axis3.value();
        int rotationPCT = Controller1.Axis1.value() * 0.25;

        if (isReversed) {
            powerPCT *= -1;
        }
        runDrive(powerPCT, rotationPCT);

        if (Controller1.ButtonUp.pressing() && !wasUpPressed) {
            // Change to forward
            isReversed = false;
            Controller1.Screen.print("FORWARD MODE!");
            Controller1.Screen.newLine();
            Controller1.rumble("...");
        } else if (Controller1.ButtonDown.pressing() && !wasDownPressed) {
            // Change to reverse
            isReversed = true;
            Controller1.Screen.print("REVERSE MODE!");
            Controller1.Screen.newLine();
            Controller1.rumble("...");
        }
        wasUpPressed = Controller1.ButtonUp.pressing();
        wasDownPressed = Controller1.ButtonDown.pressing();

        //Arm Control: X is up, Y is down
        if(Controller1.ButtonY.pressing()) { //If button up is pressed...
            //...Spin the arm motor forward.
            runArm(armSpeedPCT);
        } else if(Controller1.ButtonX.pressing()) { //If the down button is pressed...
            //...Spin the arm motor backward.
            runArm(-armSpeedPCT);
        } else { //If the the up or down button is not pressed...
            //...Brake the arm motor.
            runArm(0);
        }

        // Intake Control
        if(Controller1.ButtonL1.pressing()) { //If the upper left trigger is pressed...
            //...Spin the intake motor forward.
            runIntake(intakeSpeedPCT);
        } else if(Controller1.ButtonL2.pressing()) {
            //...Spin the intake motor backward.
            runIntake(-intakeSpeedPCT);
        } else {
            //...Stop spinning intake motor.
            runIntake(0);
        }
        wasIntakePressed = Controller1.ButtonL1.pressing();

        //Feeder Control
        if(Controller1.ButtonA.pressing()) {
            runFeeder(feederSpeedPCT);
        } else if(Controller1.ButtonB.pressing()) {
            runFeeder(-feederSpeedPCT);
        } else {
            runFeeder(0);
        }

        //Feeder Control
        if(Controller1.ButtonA.pressing()) {
            runFeeder(feederSpeedPCT);
        } else if(Controller1.ButtonB.pressing()) {
            runFeeder(-feederSpeedPCT);
        } else {
            runFeeder(0);
        }

        // Shooter Control
        currentShooterSpeedPCT = ShooterMotor.velocity(velocityUnits::pct);
        if(Controller1.ButtonR1.pressing()){
            shouldShoot = true;
        } else if (Controller1.ButtonR2.pressing()){
            shouldShoot = false;
        }
        if(shouldShoot) {
            //...Spin the shooter motor forward.
            if(ShooterMotor.velocity(velocityUnits::pct) < 90){
                powerUpShooter(currentShooterSpeedPCT);
            } else {
               Controller1.rumble(".-.");
               Controller1.Screen.print("FULL POWER REACHED");
               ShooterMotor.spin(directionType::fwd, 100, velocityUnits::pct);
           }    
        } else {
            //...Stop the shooter motor.
            if(ShooterMotor.velocity(velocityUnits::pct) > 5){
               powerDownShooter(currentShooterSpeedPCT);
            } else {
               ShooterMotor.spin(directionType::fwd, 0, velocityUnits::pct);
            }
        }

        task::sleep(30); //Sleep the task for a short amount of time to prevent wasted resources. 
    }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
    //Run the pre-autonomous function. 
    pre_auton();

    //Set up callbacks for autonomous and driver control periods.
    if (isProgrammingSkillsChallenge) {
        Competition.autonomous( programmingSkills );
    } else {
        Competition.autonomous( autonomous );
    }

    Competition.drivercontrol( usercontrol );

    //Prevent main from exiting with an infinite loop.                        
    while(1) {
      task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
    }
}
