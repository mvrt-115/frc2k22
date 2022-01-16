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

    public static class Storage{
        public static final int kBreakBeamPort = 0; 
        public static final int kMotorID = 0; // change value later
        public static final int kMotorSpeed = 0; // change value later
    }

    public static class Climber{
        //constants for the climber arms 
        // change values later
        public static final int pivotID = 0;
        public static final int leftTelescopicID = 0;
        public static final int rightTelescopicID = 0;
        public static final int limitSwitchID = 0;
        public static final int gyroID = 0;
        public static final int pivotLimitSwitchID = 0;
        public static final int teleLimitSwitchID = 0;
    }
}
