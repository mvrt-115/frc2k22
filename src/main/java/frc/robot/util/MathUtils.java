package frc.robot.util;

public class MathUtils 
{
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
     * Converts ticks to meters. 
     * 
     * @param ticks encoder tick values
     * @param ticksPerRevolution some Constants.java number
     * @param gearRatio another Constants.java number
     * @param wheelCircumference also another Constants.java number
     * @return the meters the motor(s) have traveled
     */
    public static double convertTicksToMeters(double ticks, double ticksPerRotation, double gearRatio, double wheelCircumference)
    {
        // Conversion big braining:
        // You are given ticks
        // Divide ticks by ticksPerRotation in order to get rotations
        // Divide rotations by gearRatio (aka gear to wheel ratio) in order to get wheelRotations
        // Multiply wheelRotations by wheelCircumference to get meters
        // Divide by the mass of the sun and the time it takes you to reach school in degrees Celcius if you want
        // Return meters and profit

        double rotations = ticks / ticksPerRotation;
        double wheelRotations = rotations / gearRatio;
        double meters = wheelRotations * wheelCircumference;
        return meters;
    }
}
