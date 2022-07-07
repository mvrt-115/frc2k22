// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

public class Limelight extends SubsystemBase {

  Derivitive deltaE = new Derivitive();

  private RollingAverage tx;
  private RollingAverage ty;
  private RollingAverage targetDist;
  private NetworkTable limelightTable;

  public final double height = 104; // inches
  public final double limelightMountHeight = 37.5;  // inches
  private double limelightMountAngle = 34.00001; // degrees

  public static enum LED_STATE {
    DEFAULT, ON, OFF, BLINKING;
  }

  public static enum CAM_MODE {
    VISION_WIDE, DRIVER, VISION_ZOOM;
  }

  private static Limelight limelight = new Limelight();

  public static Limelight getInstance() {
    return limelight;
  }

  /** Creates a new LimelightWrapper. */
  public Limelight() {
    
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    tx = new RollingAverage(Constants.Limelight.LIMELIGHT_ROLLING_AVG);
    ty = new RollingAverage(Constants.Limelight.LIMELIGHT_ROLLING_AVG);
    targetDist = new RollingAverage(Constants.Limelight.LIMELIGHT_ROLLING_AVG);
    
    setLED(LED_STATE.DEFAULT);
    setPipeline(CAM_MODE.VISION_WIDE);
  }

  @Override
  public void periodic() {
    // update ty and tx
    updateEntry("ty", ty);
    updateEntry("tx", tx);

    targetDist.updateValue(getDistToTarget());
    SmartDashboard.putNumber("Dist From Target", targetDist.getAverage());

    if(getHorizontalOffset() < 3 && targetsFound()) {
      Drivetrain.getInstance().setOdometry(estimatePose());
      System.out.println("hi");
    }
  }

  public void setLED(LED_STATE newState) {
    switch (newState) {
      case ON:
        limelightTable.getEntry("ledMode").setNumber(3);
        break;
      case OFF:
        limelightTable.getEntry("ledMode").setNumber(1);
        break;
      case BLINKING:
        limelightTable.getEntry("ledMode").setNumber(2);
        break;
      case DEFAULT:
        limelightTable.getEntry("ledMode").setNumber(0);
        break;
    }
  }

  public void setPipeline(CAM_MODE newMode) {
    switch (newMode) {
      case VISION_WIDE:
        limelightTable.getEntry("pipeline").setNumber(0);
        break;
      case VISION_ZOOM:
        limelightTable.getEntry("pipeline").setNumber(0);
        break;
      case DRIVER:
        limelightTable.getEntry("pipeline").setNumber(0);
        break;
    }
  }

  /**
   * Called periodically to update the rolling average values for a given entry (ty or tx)
   * @param key         The entry from limelight to change ("ty" or "tx")
   * @param rollingAvg  The rolling average to update (ty or tx)
   */
  private void updateEntry(String key, RollingAverage rollingAvg) {
    rollingAvg.updateValue((limelightTable.getEntry(key).getDouble(0)));      
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
   * Get distance to target 
   * 
   * CAP AT 200 INCHES
   * 
   * @return distance (inches)
   */
  public double getDistToTarget() {
    double angleToGoalDegrees = (90 - Constants.Limelight.MOUNT_ANGLE) + ty.getAverage();
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180);

    return (Constants.Limelight.TARGET_HEIGHT_IN - Constants.Limelight.HEIGHT_IN) / Math.tan(Math.toRadians(angleToGoalDegrees));
  }

  /**
   * Whether limelight has found any valid targets
   * 
   * @return true if targets can be found false if there aren't any
   */
  public boolean targetsFound() {
    int tv = (int) limelightTable.getEntry("tv").getDouble(0);
    if (tv == 1)
      return true;
    return false;
  }

  public Pose2d estimatePose() {

      Drivetrain dt = Drivetrain.getInstance();
      Turret turret = Turret.getInstance();

      final double fieldCenterX = 8.242;
      final double fieldCenterY = 4.051;
      final double hubRadius = 24;

      double angle = (dt.getRawGyroAngle()) + turret.getCurrentPositionDegrees();

      double distanceFromCenter = Units.inchesToMeters(targetDist.getAverage() + hubRadius);

      Pose2d pose = new Pose2d(
        fieldCenterX + Math.cos(Math.toRadians(angle)) * (distanceFromCenter), 
        fieldCenterY - Math.sin(Math.toRadians(angle)) * (distanceFromCenter), 
        dt.getGyroAngle()
      );

      SmartDashboard.putNumber("disance from center", distanceFromCenter);
      SmartDashboard.putNumber("angle", angle);
      SmartDashboard.putNumber("robot x", pose.getX());
      SmartDashboard.putNumber("robot angle", pose.getRotation().getDegrees());
      SmartDashboard.putNumber("robot y", pose.getY());

      return pose;
    }
}
