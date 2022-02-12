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
    public static final boolean kDebugMode = true;
    
    public static final int kPIDIdx = 0;
    public static final int kTimeoutMs = 10;
    public static final boolean kIsPracticeBot = true;
    public static final double kVoltageComp = 10.0;
    public static final SupplyCurrentLimitConfiguration kCurrentLimit = new SupplyCurrentLimitConfiguration(true, 40, 50, 3.8);

    public static class Drivetrain {
        // physical constants get from mech once drivetrain is done
        public static final int kTrackScrubFactor = 0;
        public static final double kTrackWidthInches = 0;

        // ids get from pheonix tuner once drivetrain is done
        public static final int kCompLeftMasterId = 1;
        public static final int kCompRightMasterId = 38;
        public static final int kCompLeftFollowerId = 3;
        public static final int kCompRightFollowerId = 4;

        public static final int kPracLeftMasterId = 20;
        public static final int kPracRightMasterId = 1;
        public static final int kPracLeftFollowerId = 34;
        public static final int kPracRightFollowerId = 15;

        // constants for joystick drive
        public static final double kSensitivity = 0.90;
        public static final double kWheelDeadband = 0.02;
        public static final double kThrottleDeadband = 0.02;
        public static final double kWheelGain = 0.05;
        public static final double kWheelNonlinearity = 0.05;

    }
    
    public static class Intake {
  
        public static final double kTicksToBottom = 0;
        // the # of ticks it takes to be at the bottom when pivoting
        public static final double kTicksToTop = 0; // the # of ticks it takes to be at the top when pivoting
      /*  public static final double PIVOT_DEPLOYED_TICKS = 900;*/

        public static final int kPivotId = 0; // change later
        public static final int kRollerId = 0; // change later

        public static final double kRollerOutput = 0.6;

        public static final double kMarginOfErrorTicks = 0;
        
        public static final double kFF = 0.31;
    }

    public static class Turret {
        public static final double kGearRatio = 47.8; // 150 * 5 * 5 / 24
        public static final double kTicksPerRevolution = 2048;
        public static final double kMinAngle = -180; // degrees
        public static final double kMaxAngle = 180; // degrees
        public static final double kLimelightOffset = 0; // degrees

        public static final double kEThreshold = 60;
        public static final double kLowETurnThreshold = 0; // degrees; limelight x fov = 59.6

        // for large angles > 60 degrees
        public static final double kPLarge = 0.5;
        public static final double kILarge = 0;
        public static final double kDLarge = 0;

        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0.005;  // try with 0.007
        public static final double kTurnSpeed = 0.3;
    }
}
