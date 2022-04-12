// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import frc.robot.util.MathUtils;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //OVERALL ROBOT CONSTANTS
    public static final boolean debugMode = true;

    public static final double MAX_VOLTAGE = 10.0;
    
    public static final int kPIDIdx = 0;
    public static final int kTimeoutMs = 10;
    public static final boolean kIsPracticeBot = true;
    public static final double kVoltageComp = 10.0;
    public static final SupplyCurrentLimitConfiguration kCurrentLimit = new SupplyCurrentLimitConfiguration(true, 40, 50, 3.8);

    public static class Drivetrain {

        
        // physical constants get from mech once drivetrain is done
        public static final double kTrackScrubFactor = 1.0469745223;
        public static final double kTrackWidthInches = 27.125; //24.85;

        // ids get from pheonix tuner once drivetrain is done
        public static int kCompLeftMasterId = 1;
        public static int kCompRightMasterId = 2;
        public static int kCompLeftFollowerId = 3;
        public static int kCompRightFollowerId = 4;
       
        public static int kPracLeftMasterId = 1;
        public static int kPracRightMasterId = 2;
        public static int kPracLeftFollowerId = 3;
        public static int kPracRightFollowerId = 4;

        // constants for joystick drive
        public static final double kSensitivity = 0.90;
        public static final double kWheelDeadband = 0.02;
        public static final double kThrottleDeadband = 0.02;
        public static final double kWheelGain = 0.05;
        public static final double kWheelNonlinearity = 0.05;
        
        //constants for getting distance traveled by motors
        public static final double kTicksPerRevolution = 2048; 
        public static final double kGearRatio = 12.78;
        
        public static final double kwheelCircumference = 6*Math.PI;  //INCHES

        public static final double kP = 0; //3.2364;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0;
                                                            
        public static final double kPTurn = 0.003;
        public static final double kITurn = 0;
        public static final double kDTurn = 0;

        //values to be determined after the robot is characterized
        public static final double kS = 0; //0.69382 //units: Volts
        public static final double kV = 1.30485; //2.6097 //units: Volts * Seconds / Meters
        public static final double kA = 0; //0.35228 //units: Volts * Seconds^2 / Meters

        public static final double IS_MOVING = 1; // Amount of m/s to say that dt motion is significant
    }
    
    public static class Intake{

        public static final double kTICKS_TO_BOTTOM = 20000;
        // the # of ticks it takes to be at the bottom when pivoting
        public static final double kTICKS_TO_TOP = 1700; // the # of ticks it takes to be at the top when pivoting


        public static final int kPIVOT_ID = 10; // change later
        public static final int kROLLER_ID = 21; // change later




        public static final double kWHEELS_SPEED = .5;



        //public static final double kPIVOT_SPEED = -0.4; // speed of intake when going up or down
        public static final double kPIVOT_STOP_SPEED_WHEN_UP = 0; // speed of intake to keep it up

        public static final double kPIVOT_STOP_SPEED_WHEN_DOWN = 0.0    ; // speed of intake to keep it down
 


        //change later after testing

        public static final double kMARGIN_OF_ERROR_TICKS = 2000; // constant


        public static final double kP = 0.0175; //0.03
        public static final double kI = 0.0;
        public static final double kD = 0.0;//0.001;

        public static final double kFF = .01;
        ;//-0.001;
    }

    public static class Limelight
    {
        public static final double MOUNT_ANGLE = 15;
        public static final double Height_IN = 36;
        public static final double TARGET_HEIGHT_IN = 104;
        public static final int LIMELIGHT_ROLLING_AVG = 8;
    }

    public static class Hood
    {
        public static final double MIN_ANG = 60;
        public static final double MAX_ANG = 80;
        public static final double P = 0.21;
        public static final double I = 0;
        public static final double D = 6.9;
        public static final double F = 0.058;
        public static final double RADIUS = 8.5;
        public static final double MAX_VOLTAGE_COMPENSATION = 5;
        public static final double GEAR_RATIO = 150.0/24;
        public static final int ENCODER_TICKS = 4096;
    }

    public static class Flywheel
    {
        public static final double P = 0.21;
        public static final double I = 0;
        public static final double D = 7.5; // 6.9
        public static final double F = 0.058;
        public static final double PHood = 0.21;
        public static final double IHood = 0;
        public static final double DHood = 6.9;
        public static final double FHood = 0.058;
        public static final double RADIUS = 2;
        public static final double ACCEPTABLE_ERROR = 25; //100;
        public static final double MAX_VOLTAGE_COMPENSATION = 10;
        public static final double TICKS_PER_REVOLUTION = 2048;
        public static final int NUM_AVG = 5;
        public static final double GEAR_RATIO = 25 / 24;
        public static final double LIN_CONST = 0;
        public static final double INCREMENT = 25; // Now it's just a constant increment not linear
        public static final double LOW_SHOT_RPM = 500;
        public static final double STRETCH_CONSTANT = 1;
        public static double REG_CONSTANT = 0;
        public static final double ADJ_HORIZ_ERROR = 0; // If the limelight is not perfectly aligning,
                                                        // this adjusts the shot for it to make.
                                                        // Only change this after shooter is tuned.
        public static final double MAX_RPM = 6380;
        public static final double ALIGN_ERROR = 3;
        public static final boolean ENMOVSHOT = false;
        public static final double OFF_TARGET = 6.75;     // How far off thwe target you want the shot to go (in inches).
    }


    public static class Actuator
    {
        public static final double THREAD_DISTANCE = 0.4; // in
        public static final double GEAR_RATIO = 56.0 / 34.0;
        public static final double TICKS_PER_ROTATION = 4096;
        public static final double HOOD_RADIUS = 11.5;
        public static final double DIST_FROM_BASE = 2 + HOOD_RADIUS;
        public static final double ACT_HEIGHT = 5;
        public static final double MAX_HEIGHT = 5;
        public static final double P = 0.05;
        public static final double I = 0;
        public static final double D = 0.19;
        public static final double kDefaultMaxServoPWM = 2.4;
        public static final double kDefaultMinServoPWM = 0.6;
        public static final double DEGREES_FROM_HORIZONTAL = 15; // How many degrees below the flywheel that the actuator is
    }

    public static class Storage
    {
        public static final int kBreakBeamPort = 0;
        public static final int kMotorID = 0;
        public static final int kMotorSpeed = 0;
    }

    public static class Turret {
        public static final double kGearRatio = 47.142857;
        public static final double kTicksPerRevolution = 2048;
        public static final double kMinAngle = -155; // -170 degrees
        public static final double kMaxAngle = 155; // 170 degrees
        public static final double kLimelightOffset = 40; // degrees
        public static final double kMaxOffset = 22;
        public static final double kEThreshold = 60;
        public static final double kLowETurnThreshold = 0; // degrees; limelight x fov = 59.6
        public static final double kTurretError = 1;
        public static final double PROP_OF_OFFSET = 0.9;

        // for large angles > 60 degrees
        public static final double kPLarge = 0.3;
        public static final double kILarge = 0;
        public static final double kDLarge = 0;

        // public static final double kP = 0.05;
        // public static final double kI = 0;
        // public static final double kD = 0.03;  // try with 0.007
        public static final double kP = 0.35;
        public static final double kI = 0;
        public static final double kD = 0.05;//-0.05;//16;
        public static final double kTurnSpeed = 0.3;
    }
        public static class Climber {
        //constants for the climber arms 
        // change values later

        //IDs/channels for sensors and motors
        //NEED TO BE CHANGED WHEN FULLY WIRED OR TESTING
        //motors
        //public static final int kPivotID = 0; // 22
        public static final int kRightTelescopicID = 6; // 6
        public static final int kLeftTelescopicID = 7; // 7

        // Servo Contants
        public static final int kRightServoID = 8; //Check (analog input) 9
        public static final int kLeftServoID = 4; // Check (analog input) 8 

        //inductive proximity sensors
        public static final int kPivotProximityChannel = 0;
        public static final int kLeftTelescopicProximityChannel = 1;
        public static final int kRightTelescopicProximityChannel = 3;
        
        //limit switch panels
        public static final int kPivotLimitSwitch = 4;
        public static final int kLeftTelescopicLimitSwitch = 6;
        public static final int kRightTelescopicLimitSwitch = 7;

        //Constants for potentiometer
        public static final int kPotentiometerPivotChannel = 0;
        public static final double kPotentiometerRange = 0;
        public static final double kPotentiometerInitialOffset = 0;

        //Math constants for calculating positions
        public static final double kTicksPerRotation = 2048;
        public static final double kPivotGearRatio = 0;
        public static final double kTelescopicGearRatio = 12 * 0.709677; //CHECK IF THIS WORKS!!!!!!!!!!!!!!!!!
        
        //PID constants for each motor
        //telescopic arm extension/retraction PID constants
        public static final double kTelekP = 0;
        public static final double kTelekI = 0;
        public static final double kTelekD = 0;
        public static final double kTelekF = 0;
        public static final double kFeedForwardTele = 0;
        
        //pivoting arm rotation PID constants
        public static final double kPivotkP = 0;
        public static final double kPivotkI = 0;
        public static final double kPivotkD = 0;
        public static final double kPivotkF = 0;
        public static final double kFeedForwardPivot = 0;

        //speeds for manual climbing
        public static final double kTelescopicRetractManualSpeed = -0.5;//-0.75; 
        public static final double kTelescopicExtendManualSpeed = 0.7;//0.8; 
        public static final double kPivotManualSpeed = 0.1;

        // servo ratchet and unratchet values
        public static final double kServoRatchet = 0; // check
        public static final double kServoUnRatchet = 0.1; // check
        public static final double kServoUnRatchet1 = 0.1; // check
        public static final double kOffsetError = 0.4; // check
        public static final double kServoError = 0.0; // check
        public static final double kMotorInitialUnratchetSpeed = 0.03; //test
        public static final double kMotorDownTime = 0.1; //seconds

        //manual position constants
        public static final double kTelescopicFullExtend = 62; //inches
        public static final double kTelescopicFullRetract = 1; //inches
        public static final double kTelescopicDownwardLimit = 50; //inches
        public static final double kTelescopicDownwardLimitTicks = MathUtils.inchesToTicks(kTelescopicDownwardLimit); //inches
        public static final double kPivotStowingPos = 32; //degrees
        public static final double kPivotMaxForwardPos = 30; //degrees
        public static final double kPivotMaxReversePos = -3; //degrees
        public static final double kApproachRungSpeed = 0.05; //PercentOutput

        //public static final double kClimberDisableTime = 30; //seconds

        //31" in height of pivot arm
        //30" min 62" max of telescopic 
        public static class Auton {
            //positions that arms move to during the climb (all pivots are in degrees, 
                //all telescopics are in meters
            // 0" on telescopic is 30"
            // 0 degrees on pivot is 90 degrees from the chassis (measured from the 14 in side of the chassis)

            // 62.64 degrees between lower and higher rung

            public static final double kTelescopicFullExtendTicks = MathUtils.inchesToTicks(kTelescopicFullExtend);
            public static final double kTelescopicFullRetractTicks = MathUtils.inchesToTicks(kTelescopicFullRetract);
            public static final double kLiftOffRungTele = MathUtils.inchesToTicks(50); // 50" (< 52.22in and > 33in)
            public static final double kPivotTeleBack = MathUtils.degreesToTicks(65); // 65 degrees
            public static final double kRotateToHighRungTele = MathUtils.degreesToTicks(62.5); // 62.5 degrees
            public static final double kHookHighRungTele = MathUtils.inchesToTicks(53); // 53" (52.22 + 0.83 (radius of rung))
            public static final double kShiftWeight = 0; //Test values??? degrees
            public static final double kRetractPivotLiftOff = MathUtils.inchesToTicks(49.15); // 49.15" (53-3.9)
            public static final double kPivotPivotingBack = MathUtils.degreesToTicks(-2); // -2 degrees
            public static final double kPivotToRung = MathUtils.degreesToTicks(2); // 2 degrees
            public static final double kRotateToHighRungPivot = MathUtils.degreesToTicks(0); // 0 degrees
            public static final double kExtendPivotHang = MathUtils.inchesToTicks(1);  // 1" 

            public static final double kAcceptablePIDError = 10; //ticks
        }
    }
}

