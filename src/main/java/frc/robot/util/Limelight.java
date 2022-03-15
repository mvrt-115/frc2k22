// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  Derivitive deltaE = new Derivitive();

  private RollingAverage tx;
  private RollingAverage ty;
  private NetworkTable limelight;

  // TODO: CHANGE THE LIMELIGHT CONSTANTS !!!!!!!!

  public final double height = 104; // inches
  public final double limelightMountHeight = 37.5;  // inches
  private double limelightMountAngle = 34.00001; // degrees

  public static enum LED_STATE {
    DEFAULT, ON, OFF, BLINKING;
  }

  public static enum CAM_MODE {
    VISION_WIDE, DRIVER, VISION_ZOOM;
  }

  /** Creates a new LimelightWrapper. */
  public Limelight() {
    
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    tx = new RollingAverage(25);
    ty = new RollingAverage(5);
    
    setLED(LED_STATE.DEFAULT);
    setPipeline(CAM_MODE.VISION_WIDE);
  }

  @Override
  public void periodic() {
    // update ty and tx
    updateEntry("ty", ty);
    updateEntry("tx", tx);
    SmartDashboard.putNumber("Dist From Target", getHorizontalDistance());
  }

  public void setLED(LED_STATE newState) {
    switch (newState) {
      case ON:
        limelight.getEntry("ledMode").setNumber(3);
        break;
      case OFF:
        limelight.getEntry("ledMode").setNumber(1);
        break;
      case BLINKING:
        limelight.getEntry("ledMode").setNumber(2);
        break;
      case DEFAULT:
        limelight.getEntry("ledMode").setNumber(0);
        break;
    }
  }

  public void setPipeline(CAM_MODE newMode) {
    switch (newMode) {
      case VISION_WIDE:
        limelight.getEntry("pipeline").setNumber(0);
        break;
      case VISION_ZOOM:
        limelight.getEntry("pipeline").setNumber(0);
        break;
      case DRIVER:
        limelight.getEntry("pipeline").setNumber(0);
        break;
    }
  }

  /**
   * Called periodically to update the rolling average values for a given entry (ty or tx)
   * @param key         The entry from limelight to change ("ty" or "tx")
   * @param rollingAvg  The rolling average to update (ty or tx)
   */
  private void updateEntry(String key, RollingAverage rollingAvg) {
    rollingAvg.updateValue((limelight.getEntry(key).getDouble(0)));      
  }

  /**
   * Get vertical angle
   * 
   * @return angle (degrees)
   */
  public double getVerticalOffset() {
    return limelightMountAngle + ty.getAverage();
  }

  /**
   * Get horizontal angle
   * 
   * @return angle (degrees)
   */
  public double getHorizontalOffset() {
    return tx.getAverage();
  }

  /**
   * Get horizontal distance
   * 
   * @return distance (meters)
   */
  public double getHorizontalDistance() {
    return getVerticalDistance() / Math.tan(Math.toRadians(getVerticalOffset()));
  }

  /**
   * Get vertical distance
   * 
   * @return height (meters)
   */
  public double getVerticalDistance() {
    return height - limelightMountHeight;
  }

  /**
   * Whether limelight has found any valid targets
   * 
   * @return true if targets can be found false if there aren't any
   */
  public boolean targetsFound() {
    int tv = (int) limelight.getEntry("tv").getDouble(0);
    if (tv == 1)
      return true;
    return false;
  }
}
