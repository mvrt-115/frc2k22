// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

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
        public static final double kTrackWidthInches = 24.85;

        // ids get from pheonix tuner once drivetrain is done
        public static int kCompLeftMasterId = 1;
        public static int kCompRightMasterId = 2;
        public static int kCompLeftFollowerId = 3;
        public static int kCompRightFollowerId = 4;
       
        public static int kPracLeftMasterId = 2;
        public static int kPracRightMasterId = 0;
        public static int kPracLeftFollowerId = 3;
        public static int kPracRightFollowerId = 1;

        // constants for joystick drive
        public static final double kSensitivity = 0.90;
        public static final double kWheelDeadband = 0.02;
        public static final double kThrottleDeadband = 0.02;
        public static final double kWheelGain = 0.05;
        public static final double kWheelNonlinearity = 0.05;

        //constants for aligning to ball from the camera
        public static final double kThrottle  = 0.0; // change value later
        public static final double kMaxPixelError = 1.0; // change value later
        
        //constants for getting distance traveled by motors
        public static final double kTicksPerRevolution = 2048;
        public static final double kGearRatio = 12.78;
        public static final double kwheelCircumference = 6*Math.PI; // inches

        public static final double kP = 1.6498;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0;

        //values to be determined after the robot is characterized
        public static final double kS = 0.5985; //units: Volts
        public static final double kV = 2.8888; //units: Volts * Seconds / Meters
        public static final double kA = 0.14018; //units: Volts * Seconds^2 / Meters

        //change values later
        public static final int kAcceptableError = 10;
        public static final double kDrivetrainMotorSpeed = 0.3;
    }
    
    public static class Intake{

        public static final double kTICKS_TO_BOTTOM = 42000;
        // the # of ticks it takes to be at the bottom when pivoting
        public static final double kTICKS_TO_TOP = -42000; // the # of ticks it takes to be at the top when pivoting


        public static final int kPIVOT_ID = 11; // change later
        public static final int kROLLER_ID = 34; // change later


        public static final double kWHEELS_SPEED = -.8;

        //public static final double kPIVOT_SPEED = -0.4; // speed of intake when going up or down
        public static final double kPIVOT_STOP_SPEED_WHEN_UP = 0; // speed of intake to keep it up

        public static final double kPIVOT_STOP_SPEED_WHEN_DOWN = 0.0    ; // speed of intake to keep it down
        //change later after testing

        public static final double kMARGIN_OF_ERROR_TICKS = 6000; // constant


        public static final double kP = 0.002;
        public static final double kI = 0.0;
        public static final double kD = 0.01;//0.001;

        public static final double kFF = .001;
        ;//-0.001;
    }

    public static class Limelight
    {
        public static final double MOUNT_ANGLE = 15;
        public static final double Height_IN = 36;
        public static final double TARGET_HEIGHT_IN = 104;
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
        public static final double D = 6.9;
        public static final double F = 0.058;
        public static final double PHood = 0.21;
        public static final double IHood = 0;
        public static final double DHood = 6.9;
        public static final double FHood = 0.058;
        public static final double RADIUS = 2;
        public static final double ACCEPTABLE_ERROR = 100;
        public static final double MAX_VOLTAGE_COMPENSATION = 10;
        public static final double TICKS_PER_REVOLUTION = 2048;
        public static final int NUM_AVG = 5;
        public static final double GEAR_RATIO = 1; // Can change
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
        public static final double kMinAngle = -180; // degrees
        public static final double kMaxAngle = 180; // degrees
        public static final double kLimelightOffset = 40; // degrees

        public static final double kEThreshold = 60;
        public static final double kLowETurnThreshold = 0; // degrees; limelight x fov = 59.6

        // for large angles > 60 degrees
        public static final double kPLarge = 0.3;
        public static final double kILarge = 0;
        public static final double kDLarge = 0;

        // public static final double kP = 0.05;
        // public static final double kI = 0;
        // public static final double kD = 0.03;  // try with 0.007
        public static final double kP = 0.3;
        public static final double kI = 0;
        public static final double kD = 0.3;//-0.05;//16;
        public static final double kTurnSpeed = 0.3;
    }
}

