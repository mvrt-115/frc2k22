package frc.robot.util;

import frc.robot.Constants;

public class MathUtils {
    /**
     * Calculates the new input by the joystick after taking into account deadband
     * 
     * @param input         raw input by the joystick
     * @param inputDeadband deadband for the value sent in
     * @return finalInput input by the joystick after calculating deadband
     */
    public static double handleDeadband(double input, double inputDeadband) {
        double finalInput = 0;

        if (Math.abs(input) < inputDeadband)
        finalInput = 0;
        else
        finalInput = calculateDeadband(input, inputDeadband);

        return finalInput;
    }

    /**
     * Calculates deadband throught an equation that allows low values to be reached
     * even after the deadband is applied.
     * 
     * @param input         original input before deadband
     * @param inputDeadband deadband being applied to the input
     * @return valAfterDeadband new input value after deadband
     */
    private static double calculateDeadband(double input, double inputDeadband) {
        double valAfterDeadband = (input - inputDeadband * Math.abs(input) / input) / (1 - inputDeadband);
        // valAfterDeadband = (1 / (1 - inputDeadband)) * (input + (Math.signum(-input)
        // * inputDeadband));
        return valAfterDeadband;
    }

    /**
     * Convert RPM to ticks per hundred milliseconds
     * @param in_rpm
     * @param ticks_per_rev
     * @param gear_ratio
     * @return ticks
     */
    public static double rpmToTicks(double in_rpm, double ticks_per_rev, double gear_ratio)
    {
        return in_rpm / 600 * ticks_per_rev * gear_ratio;
    }

    

    /**
     * Convert ticks per hundred milliseconds to RPM
     * @param ticks
     * @param ticks_per_rev
     * @param gear_ratio
     * @return rpm
     */
    public static double ticksToRPM(double ticks, double ticks_per_rev, double gear_ratio)
    {
        return ticks * 600 / ticks_per_rev / gear_ratio;
    }

    /**
     * Converts ticks to degrees
     * @param ticks         motor ticks
     * @param gearRatio     number of motor turns required to turn object once
     * @param ticksPerRev   ticks per revolution (4096 for TalonSRX, 2048 for TalonFX)
     * @return              degrees of rotation from the given ticks
     */
    public static double ticksToDegrees(double ticks, double gearRatio, double ticksPerRev) {
        return ticks / (ticksPerRev * gearRatio) * 360.0;
    }

    /**
     * convert angle to a position in ticks
     * @param degrees
     * @param encoder_ticks
     * @param gear_ratio
     * @return ticks
     */
    public static int degreesToTicks(double degrees, double encoder_ticks, double gear_ratio)
    {
        return (int) ((encoder_ticks * gear_ratio) * degrees/360);
    }
}
