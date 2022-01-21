// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Limelight;
import frc.robot.util.RollingAverage;
import frc.robot.util.TalonFactory;
import frc.robot.Constants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

public class Shooter extends SubsystemBase {

  public enum ShooterState
  {
    OFF, SPEEDING, ATSPEED;
  }

  private final double MIN_RPM = 8000;

  private final int LEADER_ID = 40;
  private final int FOLLOWER_ID = 32;
  private final int HOOD_ID = 1;

  private BaseTalon flywheelLeader;
  private BaseTalon flywheelFollower;
  private BaseTalon hoodMotor;

  // Attributes of flywheel
  private ShooterState state;
  private double targetRPM = 0;

  // Target Angle in radians
  private double targetAng = 0;

  private RollingAverage rpm;
  private Limelight limelight;

  /** Creates a new Shooter. */
  public Shooter(Limelight limelight) {
    // Mind Bending Test on the Reality of our Situation
    flywheelLeader = TalonFactory.createTalonSRX(LEADER_ID, false);
    // flywheelFollower = TalonFactory.createTalonSRX(FOLLOWER_ID, true);
    // hoodMotor = TalonFactory.createTalonSRX(HOOD_ID, false);

    // flywheelFollower.follow(flywheelLeader);

    // Sets up PIDF
    flywheelLeader.config_kP(Constants.kPIDIdx, Constants.Flywheel.P);
    flywheelLeader.config_kI(Constants.kPIDIdx, Constants.Flywheel.I);
    flywheelLeader.config_kD(Constants.kPIDIdx, Constants.Flywheel.D);
    flywheelLeader.config_kF(Constants.kPIDIdx, Constants.Flywheel.F);

    // hoodMotor.config_kP(Constants.kPIDIdx, Constants.Hood.PHood);
    // hoodMotor.config_kI(Constants.kPIDIdx, Constants.Hood.IHood);
    // hoodMotor.config_kD(Constants.kPIDIdx, Constants.Hood.DHood);
    // hoodMotor.config_kF(Constants.kPIDIdx, Constants.Hood.FHood);

    this.limelight = limelight;

    state = ShooterState.OFF;
    rpm = new RollingAverage(Constants.Flywheel.NUM_AVG);
  }

  public void stopFlywheel()
  {
    flywheelLeader.set(ControlMode.PercentOutput, 0);
  }

  public void stopHood()
  {
    hoodMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getCurrentRPM()
  {
    return rpm.getAverage();
  }

  public void setTargetRPM(double exit_velocity)
  {
    targetRPM = Math.min(MIN_RPM, exit_velocity);
    
    if(exit_velocity==0)
    {
      setState(ShooterState.OFF);
    }
    else
    {
      setState(ShooterState.SPEEDING);
    }
  }

  public void setState(ShooterState _state) {
      this.state = _state;

      if(_state == ShooterState.OFF)
      {
        targetRPM = 0;
      }
  }

  public double rpmToTicks(double in_rpm)
  {
    return in_rpm / 600 * Constants.Flywheel.TICKS_PER_REVOLUTION * Constants.Flywheel.GEAR_RATIO;
  }

  public double ticksToRPM(double ticks)
  {
    return ticks * 600 / Constants.Flywheel.TICKS_PER_REVOLUTION / Constants.Flywheel.GEAR_RATIO;
  }

  public int degreesToTicks(double degrees)
  {
    return (int) ((Constants.Hood.ENCODER_TICKS * Constants.Hood.GEAR_RATIO) * degrees/360);
  }

  public void log()
  {
    SmartDashboard.putNumber("Flywheel RPM", getCurrentRPM());
    SmartDashboard.putNumber("RPM Needed", getRequiredRPM());
    SmartDashboard.putString("State", state.toString());
    SmartDashboard.putNumber("target rpm", targetRPM);
    SmartDashboard.putNumber("target angle", targetAng);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rpm.updateValue(ticksToRPM(flywheelLeader.getSelectedSensorVelocity()));
    log();
    SmartDashboard.putNumber("time", Timer.getFPGATimestamp()); // to debug periodic

    // Sets state periodically
    switch(this.state)
    {
      case OFF:
        stopFlywheel();
        // stopHood();
        break;
      case SPEEDING:
        flywheelLeader.set(ControlMode.Velocity, rpmToTicks(targetRPM));
        if (hoodMotor != null)
          hoodMotor.set(ControlMode.Position, degreesToTicks(targetAng));
        if(allWithinError(targetRPM, Constants.Flywheel.ACCEPTABLE_ERROR))
        {
          setState(ShooterState.ATSPEED);
        }
        break;
      case ATSPEED:
        if(!allWithinError(targetRPM, Constants.Flywheel.ACCEPTABLE_ERROR))
        {
          setState(ShooterState.SPEEDING);
        }
        break;
    }
  }

  public double getRequiredRPM()
  {
    // metric values will be used until return
    double distance = Units.inchesToMeters(limelight.getHorizontalDistance());

    // distance from center of hub
    double dx = distance + Units.inchesToMeters(24);

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
    
    return Units.metersToInches(60) * vel_proj / (Constants.Flywheel.RADIUS * 2 * Math.PI);
  }

  public double getRequiredAng()
  {
    // metric values will be used until return
    double distance = limelight.getHorizontalDistance();

    // distance from center of hub
    double dx = distance + Units.inchesToMeters(24);

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

  /**
   * @return BaseTalon flywheel leader
   */
  public BaseTalon getMotor()
  {
    return flywheelLeader;
  }
}
