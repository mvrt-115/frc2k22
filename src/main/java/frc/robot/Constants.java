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

        //constants for aligning to ball from the camera
        public static final double kThrottle  = 0.0; // change value later
        public static final double kMaxPixelError = 1.0; // change value later
        
        //constants for getting distance traveled by motors
        public static final double kTicksPerRevolution = 1.0; //RANDOM VALUES
        public static final double kGearRatio = 10.0;
        public static final double kwheelCircumference = 5.0;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFF = 0;

        //values to be determined after the robot is characterized
        public static final double kS = 0.1; //units: Volts
        public static final double kV = 0.1; //units: Volts * Seconds / Meters
        public static final double kA = 0.1; //units: Volts * Seconds^2 / Meters

        //change values later
        public static final int kAcceptableError = 10;
        public static final double kDrivetrainMotorSpeed = 0.5;
    }
    
    public static class Intake{

        public static final double kTicksToBottom = 0;
        // the # of ticks it takes to be at the bottom when pivoting
        public static final double kTicksToTop = 0; // the # of ticks it takes to be at the top when pivoting

        public static final int kPivotID = 0; // change later
        public static final int kRollerID = 0; // change later

        public static final double kWheelSpeed = 0.6;

        public static final double kPivotSpeed = 0.4; // speed of intake when going up or down
        public static final double kPivotStopSpeedWhenUp = 0.23; // speed of intake to keep it up
        public static final double kPivotStopSpeedWhenDown = 0.0; // speed of intake to keep it down
        //change later after testing

        public static final double kPivotSpeedWhenDown = 0;
        public static final double kPivotSpeedWhenUp = 0;
        public static final double kMarginOfErrorTicks = 0;

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
}