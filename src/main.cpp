#include "robot-config.h"
#include <cmath>
/*---------------------------------------------------------------------------                                        
        Description: Team 2585's VEX Control Software for 2018-19          
---------------------------------------------------------------------------*/

//Creates a competition object that allows access to Competition methods.
competition    Competition;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  Performs functions before the game starts. You must return from this     */
/*  or the autonomous and usercontrol tasks will not be started. This        */
/*  function is only called once after the cortex has been powered on and    */ 
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
void pre_auton( void ) {
    for(motor each: driveTrainMotors){
        each.resetRotation();
    }
    return;
}

// Robot measurements
const float wheelDiameter = 4.125; // inches
const float turningDiameter = 17.5; //inches (TODO: re-measure horizontally)
const float gearRatio = 1.0; // 1 turns of motor/<gearRatio> turns of wheel
const float wheelCircumference = wheelDiameter * 3.141592653589;
const float inchesPerDegree = wheelCircumference / 360.0;
// should be turningDiameter/wheelDiameter
// Didn't work so 3.337 measured experimentally
const float encoderTicksPerDegree = 3.337;

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

void activeBrake(){
    for(motor each: driveTrainMotors){
        each.stop(brakeType::hold);
    }
}

void runDriveTank(float leftvL, float rightvL, bool isReversed) {
    if (isReversed) {
        leftvL *= -1;
        rightvL *= -1;
    }
    LBMotor.spin(directionType::fwd, leftvL, voltageUnits::volt);
    LFMotor.spin(directionType::fwd, leftvL, voltageUnits::volt);
   
    RBMotor.spin(directionType::fwd, rightvL, voltageUnits::volt);
    RFMotor.spin(directionType::fwd, rightvL, voltageUnits::volt);
}

void runDriveArcade(float powervL, float rotationvL, bool isReversed) {
    //positive rotation --> turning right
    //negative rotation --> turning left
    powervL = (isReversed) ? powervL * -1 : powervL;
    runDriveTank(powervL + rotationvL, powervL - rotationvL, false);
}

void autoDriveForward( float inches, float power ) { // distance in inches
    float degreesTurn = inches / inchesPerDegree * gearRatio;

    LeftBackMotor.startRotateFor(degreesTurn, rotationUnits::deg, power, velocityUnits::pct);
    LeftFrontMotor.startRotateFor(degreesTurn, rotationUnits::deg, power, velocityUnits::pct);

    RightBackMotor.startRotateFor(degreesTurn, rotationUnits::deg, power, velocityUnits::pct);
    RightFrontMotor.rotateFor(degreesTurn, rotationUnits::deg, power, velocityUnits::pct);
}  

void autoDriveForwardRaw(float power, float time){
    for(motor each: driveTrainMotors){
        each.spin(directionType::fwd, power, velocityUnits::pct);
    }

    task::sleep(time * 1000); //time(in seconds) to milliseconds

    for(motor each: driveTrainMotors){
        each.stop(brakeType::brake);
    }

}

void autoTurn( float degrees ) {
    // Note: +90 degrees is a right turn
    float encoderDegrees = encoderTicksPerDegree * degrees;
    LeftBackMotor.startRotateFor(encoderDegrees, rotationUnits::deg, 50, velocityUnits::pct);
    LeftFrontMotor.startRotateFor(encoderDegrees, rotationUnits::deg, 50, velocityUnits::pct);

    RightBackMotor.startRotateFor(-encoderDegrees, rotationUnits::deg, 50, velocityUnits::pct);
    RightFrontMotor.rotateFor(-encoderDegrees, rotationUnits::deg, 50, velocityUnits::pct);
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
                runDriveArcade(0, -10, false);
            } else if (VisionSensor.largestObject.centerX > screenMiddleX + 5) {
                //on the right, turn right
                runDriveArcade(0, 10, false);
            } else {
                //in the middle, we're done lining up
                isLinedUp = true;
                runDriveArcade(0, 0, false);
            }
        } else {
            return;
        }
    }
    return;
}

void pointTo(vision::code sig) {
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
                runDriveArcade(0, -10, false);
            } else if (VisionSensor.largestObject.centerX > screenMiddleX + 5) {
                //on the right, turn right
                runDriveArcade(0, 10, false);
            } else {
                //in the middle, we're done lining up
                isLinedUp = true;
                runDriveArcade(0, 0, false);
            }
        } else {
            return;
        }
    }
    return;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control the robot during the autonomous phase of    */
/*  a VEX Competition.                                                       */
/*---------------------------------------------------------------------------*/
void autonomous( void ) {
    if(isFront){
        //Charge up shooter
        ShooterMotor.spin(directionType::fwd, 100, velocityUnits::pct);
        
        //Hit low flag
        autoDriveForward(48, 90);
        autoDriveForwardRaw(50, 0.25);
        
        //Hit high flag
        autoDriveForward(-16.5, 100);
        if(isBlue){
            autoTurn(-15);
        } else {
            autoTurn(15);
        }
        
        IntakeMotor.startRotateFor(100, rotationUnits::rev, 100, velocityUnits::pct);
        Feeder.startRotateFor(10, rotationUnits::rev, 20, velocityUnits::pct);
        task::sleep(2000);
        Feeder.stop(brakeType::coast);
        IntakeMotor.stop(brakeType::coast);
        ShooterMotor.stop(brakeType::coast);
        if(isBlue){
            autoTurn(15);
        } else {
            autoTurn(-15);
        }
        
        autoDriveForward(-8.5, 75);
        
        //Flip cap
        if(isBlue){
            autoTurn(-90);
        } else {
            autoTurn(90);
        }
        autoDriveForwardRaw(-50, 0.5);
        autoDriveForward(12, 75);
        IntakeMotor.startRotateFor(-100, rotationUnits::rev, 100, velocityUnits::pct);
        autoDriveForward(12, 25);
        IntakeMotor.stop(brakeType::coast);
        
        autoDriveForward(-24, 100);
        autoDriveForwardRaw(-50, 0.5);
        autoDriveForward(8, 75);
        
        if(isBlue){
            autoTurn(-90);
        } else {
            autoTurn(90);
        }
        
        autoDriveForward(18, 100);
        if(isBlue){
            autoTurn(90);
        } else {
            autoTurn(-90);
        }
        autoDriveForwardRaw(-50, 0.25);
        
        autoDriveForward(24, 100);
        IntakeMotor.startRotateFor(100, rotationUnits::rev, 100, velocityUnits::pct);
        autoDriveForward(18, 50);
        task::sleep(500);
        IntakeMotor.stop(brakeType::coast);
        autoDriveForward(-12, 75);
        if(isBlue){
            autoTurn(90);
        } else {
            autoTurn(-90);
        }
        //Hit flag
        autoDriveForward(-12, 75);
        autoDriveForward(-48, 100);
    } else { // in the back
        //flip cap
        ShooterMotor.spin(directionType::fwd, 100, velocityUnits::pct);
        autoDriveForward(24, 50);
       
        //Intake ball
        IntakeMotor.startRotateFor(100, rotationUnits::rev, 100, velocityUnits::pct);
        autoDriveForward(18, 50);
        task::sleep(500);
        
        //Reset position
        IntakeMotor.stop(brakeType::coast);
        autoDriveForward(-42, 50);
        autoDriveForwardRaw(-25, 0.9);
        autoDriveForward(6, 50);
        if(isBlue){
            autoTurn(90);
        } else {
            autoTurn(-90);
        }
        
        IntakeMotor.startRotateFor(100, rotationUnits::rev, 100, velocityUnits::pct);
        Feeder.startRotateFor(10, rotationUnits::rev, 20, velocityUnits::pct);
        task::sleep(1000);
        Feeder.stop(brakeType::coast);
        autoDriveForward(24, 50);
        Feeder.startRotateFor(10, rotationUnits::rev, 40, velocityUnits::pct);
        task::sleep(1500);
        IntakeMotor.stop(brakeType::coast);
        Feeder.stop(brakeType::coast);
        ShooterMotor.stop(brakeType::coast);
        
        //Climb platform
        if(isBlue){
            autoTurn(90);
        } else {
            autoTurn(-90);
        }
        autoDriveForwardRaw(25, 0.5);
        autoDriveForward(-65, 100); //park to alliance platform
    }
    return;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*---------------------------------------------------------------------------*/
void usercontrol( void ) {
    pre_auton();
    //Use these variables to set the speed of the arm, intake, and shooter.
    int armSpeedPCT = 20;
    int intakeSpeedPCT = 100;
    int feederSpeedPCT = 60;

    Controller1.Screen.print("Welcome Aboard!");
    Controller1.Screen.newLine();
    Controller1.Screen.print("Get ready to rumble!!!");
    Controller1.Screen.newLine();

    Controller1.Screen.print("FORWARD MODE!");
    Controller1.Screen.newLine();
    Controller1.rumble("--..-");

    // Keep track of past button state to detect inital presses
    bool isReversed = false;
    bool shouldShoot = false;
    bool wasUpPressed = false;
    bool wasDownPressed = false;
    
    while (1) {
        // This is the main execution loop for the user control program.
        // Each time through the loop your program should update motor + servo
        
        //Drive Control
        // Arcade Control
        float powervL = Controller1.Axis3.value() * 12.0 / 127;
        
        float rotationvL = 0.3 * Controller1.Axis1.value() * 12.0 / 127;
        float slowRotationvL = 0.15 * Controller1.Axis4.value() * 12.0 / 127;
        
        // Tank Control
        float leftvL = powervL;
        float rightvL = Controller1.Axis2.value() * 12.0 / 127;
        
        if(std::abs(slowRotationvL) < 5){
            slowRotationvL = 0;
        } else {
           if(std::abs(rotationvL) < 5)
                rotationvL = slowRotationvL;
        }
        if(rotationvL != 0){
            runDriveArcade(powervL, rotationvL, isReversed);
        } else {
            runDriveTank(leftvL, rightvL, isReversed);
        }
        
        //Active brake
        if(Controller1.ButtonA.pressing()){
            activeBrake();
        }
        
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
        /*
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
        */

        //Feeder Control
        if(Controller1.ButtonA.pressing()) {
            runFeeder(feederSpeedPCT);
        } else if(Controller1.ButtonB.pressing()){
            runFeeder(-feederSpeedPCT);
        } else {
            runFeeder(0);
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
        
        // Shooter Control
        if(Controller1.ButtonR1.pressing()){
            shouldShoot = true;
        } else if (Controller1.ButtonR2.pressing()){
            shouldShoot = false;
        }
        if(shouldShoot) {
            //...Spin the shooter motor forward.
            ShooterMotor.spin(directionType::fwd, 100, velocityUnits::pct);
            if(ShooterMotor.velocity(velocityUnits::pct) > 90){
                Controller1.Screen.print("FULL POWER REACHED");
                Controller1.rumble("-.-");
            }
        } else {
            //...Stop the shooter motor.
            ShooterMotor.spin(directionType::fwd, 0, velocityUnits::pct);
        }
        
        task::sleep(30); //Sleep the task for a short amount of time to prevent wasted resources. 
    }
}

// Main will set up the competition functions and callbacks.
int main() {
    //Run the pre-autonomous function. 
    pre_auton();
    //Set up callbacks for autonomous and driver control periods.
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );

    //Prevent main from exiting with an infinite loop.                        
    while(1) {
      task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
    }
}
