// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Limelight;
import frc.robot.util.RollingAverage;
import frc.robot.Constants;
import frc.robot.util.MathUtils;
import frc.robot.util.TalonFactory;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

public class Shooter extends SubsystemBase
{

  private boolean shoot;

  private final int LEADER_ID = 0;
  private final int FOLLOWER_ID = 0;
  private final int HOOD_ID = 0;

  private BaseTalon flywheelLeader;
  private BaseTalon flywheelFollower;
  private BaseTalon hoodMotor;

  // Attributes of flywheel
  private double targetRPM = 0;

  // Target Angle in radians
  private double targetAng = 0;

  private RollingAverage rpm;
  private Limelight limelight;

  /** Creates a new Shooter. */
  public Shooter(Limelight limelight)
  {

    shoot = false;

    // Mind Bending Test on the Reality of our Situation
    if (RobotBase.isReal())
    {
      flywheelLeader = TalonFactory.createTalonFX(LEADER_ID, false);
      flywheelFollower = TalonFactory.createTalonFX(FOLLOWER_ID, false);
      hoodMotor = TalonFactory.createTalonFX(HOOD_ID, false);

      flywheelLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      flywheelFollower.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
      hoodMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }

    // Hood Motor Zeroed at an angle 60 degrees to horizontal
    hoodMotor.setSelectedSensorPosition(0);

    flywheelLeader.configVoltageCompSaturation(Constants.MAX_VOLTAGE);
    flywheelFollower.configVoltageCompSaturation(Constants.MAX_VOLTAGE);
    hoodMotor.configVoltageCompSaturation(Constants.MAX_VOLTAGE);

    flywheelLeader.setInverted(false);
    flywheelFollower.setInverted(false);
    hoodMotor.setInverted(false);

    flywheelFollower.follow(flywheelLeader);

    // Sets up PIDF
    flywheelLeader.config_kP(Constants.kPIDIdx, Constants.Flywheel.P);
    flywheelLeader.config_kI(Constants.kPIDIdx, Constants.Flywheel.I);
    flywheelLeader.config_kD(Constants.kPIDIdx, Constants.Flywheel.D);
    flywheelLeader.config_kF(Constants.kPIDIdx, Constants.Flywheel.F);

    hoodMotor.config_kP(Constants.kPIDIdx, Constants.Hood.P);
    hoodMotor.config_kI(Constants.kPIDIdx, Constants.Hood.I);
    hoodMotor.config_kD(Constants.kPIDIdx, Constants.Hood.D);
    hoodMotor.config_kF(Constants.kPIDIdx, Constants.Hood.F);

    this.limelight = limelight;

    targetRPM = 0;
    rpm = new RollingAverage(Constants.Flywheel.NUM_AVG);
  }

  public double getCurrentRPM()
  {
    return rpm.getAverage();
  }

  public void stopFlywheel()
  {
    flywheelLeader.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    rpm.updateValue(MathUtils.ticksToRPM(flywheelLeader.getSelectedSensorVelocity(),
        Constants.Flywheel.TICKS_PER_REVOLUTION, Constants.Flywheel.GEAR_RATIO));
    targetAng = getRequiredAng();

    // Sets state periodically
    if(shoot)
    {
      flywheelLeader.set(ControlMode.Velocity, MathUtils.rpmToTicks(targetRPM, 
          Constants.Flywheel.GEAR_RATIO));

      log("Current RPM", getCurrentRPM());
      log("Desired RPM", targetRPM);
      log("Shoot", String.valueOf(shoot));
    }
    else
    {
      stopFlywheel();
    }

    // Adjust Hood continuously while not shooting 
    if(!shoot)
    {
      hoodMotor.set(ControlMode.Position, MathUtils.degreesToTicks(targetAng, 
            Constants.Hood.ENCODER_TICKS, Constants.Flywheel.GEAR_RATIO));
    }
  }

  public void setShot(boolean _shoot)
  {
    if(_shoot)
    {
      shoot = true;
      targetRPM = getRequiredRPM();

      log("Target RPM", targetRPM);
      log("Target Angle", targetAng);
    }
    else
    {
      shoot = false;
    }
  }

  public boolean isReady()
  {
    return allWithinError(targetRPM, Constants.Flywheel.ACCEPTABLE_ERROR);
  }

  public double getRequiredRPM()
  {
    // metric values will be used until return
    double distance = limelight.getDistanceFromTarget()/Constants.Flywheel.METERS_TO_IN;

    // distance from center of hub
    double dx = distance + 24/39.3701;

    // function to get velocity value given shooter height is 3ft. from the ground
    double vel_proj = 0;

    if(dx<=1.72)
    {
      vel_proj = -12.186 * Math.pow(dx, 3) + 54.736 * Math.pow(dx, 2) - 81.631 * dx + 48.166;
    }
    else
    {
      vel_proj = 0.2546 * Math.pow(dx, 2) - 0.0295 * dx + 7.0226;
    }
    
    return 60 * vel_proj * Constants.Flywheel.METERS_TO_IN / (Constants.Flywheel.RADIUS * 2 * Math.PI);
  }

  public double getRequiredAng()
  {
    // metric values will be used until return
    double distance = limelight.getDistanceFromTarget()/Constants.Flywheel.METERS_TO_IN;

    // distance from center of hub
    double dx = distance + 24/39.3701;

    // function to get theta value
    double angle_proj = 0;

    // Set the angle from 0 to 20 (60 to 80) based on the distance
    if(dx<=2.78)
    {
      angle_proj = 20 - (5.6845 * Math.pow(dx, 2) - 33.067 * dx + 109.07 - 60);
      
      if(angle_proj>20)
      {
        angle_proj = 20;
      }
      else if(angle_proj<0)
      {
        angle_proj = 0;
      }
    }
    else
    {
      angle_proj = 20;
    }

    return angle_proj;
  }

  /**
     * @param target -- the target RPM
     * @param acceptableError -- the acceptable +- error range
     * @return boolean whether the RPM is within the acceptable error or not
     */
    private boolean allWithinError(double target, double acceptableError) {
      return Math.abs(rpm.getAverage() - target) <= acceptableError;
  }

  public void log(String name, double value)
  {
    SmartDashboard.putNumber(name, value);
  }

  public void log(String name, String text)
  {
    SmartDashboard.putString(name, text);
  }
}
