package frc.robot.util;



import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
     * Converts ticks to meters. 
     * 
     * @param ticks encoder tick values
     * @param ticksPerRevolution some Constants.java number
     * @param gearRatio another Constants.java number
     * @param wheelCircumference also another Constants.java number
     * @return the meters the motor(s) have traveled
     */
    public static double convertTicksToMeters(double ticks, double ticksPerRotation, double gearRatio, double wheelCircumference) {
        // Conversion big braining:
        // You are given ticks
        // Divide ticks by ticksPerRotation in order to get rotations
        // Divide rotations by gearRatio (aka gear to wheel ratio) in order to get wheelRotations
        // Multiply wheelRotations by wheelCircumference to get meters
        // Divide by the mass of the sun and the time it takes you to reach school in degrees Celcius if you want
        // Return meters and profit

        double rotations = ticks / ticksPerRotation;
        double wheelRotations = rotations / gearRatio;
        double meters = wheelRotations * Units.inchesToMeters(wheelCircumference);
        return meters;
    }
    // public static double metersToTicks(double meters){

    //     double rotations = meters  / (2*Math.PI* inchesToMeters(Constants.Drivetrain.kwheelCircumference)) * (Constants.Drivetrain.kGearRatio);
    //     return rotations * 2048;
        
    // }

    public static double metersToRadians(double wheelCircumfrence, double meters){
        double rotations = meters / Units.inchesToMeters(wheelCircumfrence);
        SmartDashboard.putNumber("Rotations", rotations);
        return rotations * 2 * Math.PI;
    }

    /** 
     * Converts RPM to meters/second using some big brain conversions
     * @param ticksPer100ms speed in ticks per 100 milliseconds
     * @param ticksPerRevolution some Constants.java number
     * @param gearRatio another Constants.java number
     * @param wheelCircumference also another Constants.java number
     * @return metersPerSecond speed in m/s (metric system ftw)
     */
    public static double RPMtoMetersPerSecond(double ticksPer100ms, double ticksPerRotation, double gearRatio, double wheelCircumference) {
        // Conversion big braining:
        // You are given ticks per 100 ms (or 0.1 s)
        // Multiply by 10 in order to get ticks per second
        // Divide by ticksPerRotation constant to get gear rotations per second
        // Divide by gear ratio to get wheel rotations per second
        // Multiply wheel rotations per second by wheel circumeference (in meters) to get meters per second
        // Divide by the mass of the sun in Kelvin
        // Return meters per second and profit

        double ticksPerSecond = ticksPer100ms * 10; //100 ms = 0.1 s, 10 * 100 ms = 1 second
        double gearRotationsPerSecond = ticksPerSecond / ticksPerRotation;
        double wheelRotationsPerSecond = gearRotationsPerSecond / gearRatio;
        double metersPerSecond = wheelRotationsPerSecond * Units.inchesToMeters(wheelCircumference);
        return metersPerSecond;
    }

    public static double inchesToMeters(double inches) {
        return inches * 0.0254; //no way it's Team 254 :O
    }
    public static double rpmToTicks(double in_rpm, double gear_ratio)
    {
        return in_rpm / 600 * Constants.Flywheel.TICKS_PER_REVOLUTION * gear_ratio;
    }

    public static double ticksToRPM(double ticks, double ticks_per_rev, double gear_ratio)
    {
        return ticks * 600 / ticks_per_rev / gear_ratio;
    }

    public static int degreesToTicks(double degrees, double encoder_ticks, double gear_ratio)
    {
        return (int) ((encoder_ticks * gear_ratio) * degrees/360);
    }
}
