using namespace vex;
brain Brain;
controller Controller1 = controller();

namespace vex {
  class motor_special : public motor {
    private:
      uint32_t  _local_index;

    public:
      motor_special( int32_t index, bool isReversed ) :  motor( index, isReversed ), _local_index(index) {};  
      ~motor_special() {};

      // Allow overloading of base class methods
      using motor::spin;
      
      // This is similar, not quite the same, as new a method in the next (Nov ?) SDK release
      // voltage can be +/-12.0 volta or +/-12000 mV
      void spin( directionType dir, double voltage, voltageUnits units ) {
        // convert volts to mV is necessary
        int32_t voltage_mv = (units == voltageUnits::volt ? voltage * 1000.0 : voltage );

        // flip based on direction flag
        voltage_mv = (dir == directionType::fwd ? voltage_mv : -(voltage_mv) );

        if( voltage_mv == 0 ) {
          stop();
        } else {
          // send mV value to control motor open loop
          vexMotorVoltageSet( _local_index, voltage_mv );
        }
      }
  };
}

//Port declarations
motor_special LBMotor = motor_special(PORT1, false);
motor_special LFMotor = motor_special(PORT2, false);
motor_special RBMotor = motor_special(PORT10, true);
motor_special RFMotor = motor_special(PORT9, true);

motor LeftBackMotor = motor(PORT1, false);
motor LeftFrontMotor = motor(PORT2, false);
motor RightBackMotor = motor(PORT10, true);
motor RightFrontMotor = motor(PORT9, true);
motor driveTrainMotors[] = {LeftBackMotor, LeftFrontMotor, RightBackMotor, RightFrontMotor};
motor ArmMotor = motor(PORT11, true);
motor Feeder = motor(PORT20, true);
motor IntakeMotor = motor(PORT3);
motor ShooterMotor = motor(PORT5, true);

//Vision Setup
vision::signature SIG_1 (1, 0, 0, 0, 0, 0, 0, 3, 0);
vision::signature BLUE_OBJ (2, -2979, 381, -1299, -1, 10427, 5213, 0.30000001192092896, 1);
vision::signature RED_OBJ (3, 4131, 11843, 7987, -3099, -1531, -2315, 1, 1);
vision::signature GREEN_FLAG (4, -3521, -2421, -2971, -7583, -6733, -7158, 1.7000000476837158, 1);
vision::signature SIG_5 (5, 0, 0, 0, 0, 0, 0, 3, 0);
vision::signature SIG_6 (6, 0, 0, 0, 0, 0, 0, 3, 0);
vision::signature SIG_7 (7, 0, 0, 0, 0, 0, 0, 3, 0);
vision::code BLUE_FLAG (BLUE_OBJ, GREEN_FLAG);
vision::code RED_FLAG (RED_OBJ, GREEN_FLAG);
vision VisionSensor = vision(PORT5, 50, SIG_1, BLUE_OBJ, RED_OBJ, GREEN_FLAG, SIG_5, SIG_6, SIG_7);

// Robot starting position
const bool isBlue = false;
const bool isFront = true;
