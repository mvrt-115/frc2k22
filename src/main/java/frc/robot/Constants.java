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
    public static final boolean kIsPracticeBot = false;
    public static final double kVoltageComp = 10.0;
    public static final SupplyCurrentLimitConfiguration kCurrentLimit = new SupplyCurrentLimitConfiguration(true, 40, 50, 3.8);

    public static class Drivetrain {

        // physical constants get from mech once drivetrain is done
        public static final int kTrackScrubFactor = 0;
        public static final double kTrackWidthInches = 0;

        // ids get from pheonix tuner once drivetrain is done
        public static int kCompLeftMasterId = 1;
        public static int kCompRightMasterId = 2;
        public static int kCompLeftFollowerId = 3;
        public static int kCompRightFollowerId = 4;
        public static int kPracLeftMasterId = 5;
        public static int kPracRightMasterId = 6;
        public static int kPracLeftFollowerId = 7;
        public static int kPracRightFollowerId = 8;

        // constants for joystick drive
        public static final double kSensitivity = 0.90;
        public static final double kWheelDeadband = 0.02;
        public static final double kThrottleDeadband = 0.02;
        public static final double kWheelGain = 0.05;
        public static final double kWheelNonlinearity = 0.05;

    }
    
    public static class Intake{

        public static final double kTICKS_TO_BOTTOM = 0;
        // the # of ticks it takes to be at the bottom when pivoting
        public static final double kTICKS_TO_TOP = 0; // the # of ticks it takes to be at the top when pivoting

        public static final int kPIVOT_ID = 0; // change later
        public static final int kROLLER_ID = 0; // change later

        public static final double kWHEELS_SPEED = 0.6;

        public static final double kPIVOT_SPEED = 0.4; // speed of intake when going up or down
        public static final double kPIVOT_STOP_SPEED_WHEN_UP = 0.23; // speed of intake to keep it up
        public static final double kPIVOT_STOP_SPEED_WHEN_DOWN = 0.0; // speed of intake to keep it down
        //change later after testing

        public static final double kMARGIN_OF_ERROR_TICKS = 0;

        // change values later
        public static final double kP = 0.0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0.31;
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
        public static final double P = 0.83;
        public static final double I = 0;
        public static final double D = 10.5;
        public static final double F = 0.058;
        public static final double PHood = 0.21;
        public static final double IHood = 0;
        public static final double DHood = 6.9;
        public static final double FHood = 0.058;
        public static final double RADIUS = 2;
        public static final double ACCEPTABLE_ERROR = 50;
        public static final double MAX_VOLTAGE_COMPENSATION = 10;
        public static final double TICKS_PER_REVOLUTION = 4096;
        public static final int NUM_AVG = 5;
        public static final double GEAR_RATIO = 50.0/36.0; // Can change
    }

    public static class Storage
    {
        public static final int kBreakBeamPort = 0;
        public static final int kMotorID = 0;
        public static final int kMotorSpeed = 0;
    }
}
