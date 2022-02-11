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
        if (Math.abs(input) < inputDeadband)
            return 0;
        else
            return calculateDeadband(input, inputDeadband);
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
   * Converts degrees to ticks
   * @param degrees an angle in degrees
   * @return        the amount of ticks in that angle
   */
  public static double degreesToTicks(double degrees) {
    return (degrees / 360) * Constants.Climber.kTicksPerRotation * Constants.Climber.kPivotGearRatio;
  }

  /**
   * Convert the ticks to degrees
   *@param ticks the amount of ticks rotated
   *@return The amount of degrees in ticks
   ticks * rotation/ticks * rotation motor/rotation arm * degrees/rotation = degrees of arm
   Follow this calculation
   */
  public static double ticksToDegrees(double ticks) {
    return (ticks / Constants.Climber.kTicksPerRotation / Constants.Climber.kPivotGearRatio) * 360;
  }

  /**
   * Converts ticks to meters
   * @param ticks The ticks to convert
   * @return The tick value converted to meters 
   */
  public static double ticksToMeters(double ticks) {
    return ticks;
  }

  /**
   * Converts meters to ticks
   * @param meters a distance in meters
   * @return        the amount of ticks in that distance
   */
  public static double metersToTicks(double meters) {
    return meters;
  }

  /**
   * Converts inches to meters
   * @param inches a distance inches
   * @return        the amount of meters in that distance
   */
  public static double inchesToMeters(double inches)
  {
    return inches * 2.54 / 100;
  }

  /**
   * Converts inches to meters
   * @param inches a distance inches
   * @return        the amount of meters in that distance
   */
  public static double metersToInches(double meters)
  {
    return meters / 100 * 2.54;
  }

  /**
   * Converts inches to meters
   * @param inches a distance inches
   * @return        the amount of meters in that distance
   */
  public static double inchesToTicks(double inches)
  {
    return metersToTicks(inchesToMeters(inches));
  }

  public static double ticksToInches(double ticks)
  {
    return metersToInches(ticksToMeters(ticks));
  }
}
