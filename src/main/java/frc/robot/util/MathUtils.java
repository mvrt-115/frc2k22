package frc.robot.util;

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
     * Converts ticks to degrees
     * @param angle         angle of the object
     * @param gearRatio     number of motor turns required to turn object once
     * @param ticksPerRev   ticks per revolution (4096 for TalonSRX, 2048 for TalonFX)
     * @return              number of ticks equivalent to angle
     */
    public static double degreesToTicks(double angle, double gearRatio, double ticksPerRev) {
        return angle * (ticksPerRev * gearRatio) / 360.0;
    }
}
