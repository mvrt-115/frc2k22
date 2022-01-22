// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Derivitive;
import frc.robot.util.Limelight;
import frc.robot.util.MathUtils;
import frc.robot.util.TalonFactory;

public class Turret extends SubsystemBase {

  public static enum TurretState {
    TARGETING, FLIPPING, CAN_SHOOT, DISABLED
  }

  private TurretState state;

  private TalonSRX turret, left, right;

  private Limelight limelight;

  private double targetDegrees;

  // AHRS gyro = new AHRS(SPI.Port.kMXP);

  // double gyroInitial = 0;

  // rate of change of angle
  private Derivitive deltaE;

  /** Creates a new Turret. */
  public Turret(Limelight limelight) {
    this.limelight = limelight;

    turret = TalonFactory.createTalonSRX(42, false);
    left = TalonFactory.createTalonSRX(38, false);
    right = TalonFactory.createTalonSRX(1, true);

    targetDegrees = 0;

    state = TurretState.DISABLED;

    deltaE = new Derivitive();

    turret.config_kP(0, Constants.Turret.kP);
    turret.config_kI(0, Constants.Turret.kI);
    turret.config_kD(0, Constants.Turret.kD);

    turret.config_kP(1, Constants.Turret.kPLarge);
    turret.config_kI(1, Constants.Turret.kILarge);
    turret.config_kD(1, Constants.Turret.kDLarge);

    turret.setSelectedSensorPosition(0);
    turret.selectProfileSlot(0, 0);
  }

  @Override
  public void periodic() {
    if(state != TurretState.DISABLED)
     return;

    // determine if we can shoot if we are within some margin of error
    if(Math.abs(getCurrentPositionDegrees() - targetDegrees) <= 5)
      setState(TurretState.CAN_SHOOT);
    else
      setState(TurretState.TARGETING);

    // continue looking for target
    if(state == TurretState.FLIPPING) {
      turnToTarget();

      if(Math.abs(getCurrentPositionDegrees()) <= 10)
        setState(TurretState.TARGETING);
    } else if(state != TurretState.DISABLED) {
       target();
    } else {
      turnPercentOut(0);
    }

    // left.set(ControlMode.PercentOutput, 0.7);
    // right.set(ControlMode.PercentOutput, -0.7);

    log();
  }

  /**
   * Moves the turret to lock onto the target. If a target has been found, the turret will move there. Otherwise,
   * the turret will turn until it does see something.
   */
  public void target() {
    if(limelight.targetsFound()) {
      // find target position by using current position and data from limelight
      targetDegrees = getCurrentPositionDegrees() + limelight.getHorizontalOffset();

      deltaE.update(limelight.getHorizontalOffset());

      if(targetDegrees > Constants.Turret.kMaxAngle) {
        setState(TurretState.FLIPPING);

        targetDegrees = Constants.Turret.kMinAngle + targetDegrees - Constants.Turret.kMaxAngle;
      } else if(targetDegrees < Constants.Turret.kMinAngle) {
        setState(TurretState.FLIPPING);

        targetDegrees = Constants.Turret.kMaxAngle + targetDegrees - Constants.Turret.kMinAngle;
      }

      turnToTarget();
    } else {
      turnPercentOut(0.4);

      if(getCurrentPositionDegrees() > Constants.Turret.kMaxAngle || getCurrentPositionDegrees() < Constants.Turret.kMinAngle)
        turnPercentOut(-turret.getMotorOutputPercent());
    }
  }

  /**
   * Turns to the desired degrees
   */
  private void turnToTarget() {
    changePIDSlot(getCurrentPositionDegrees() - targetDegrees);

    turnPos(MathUtils.degreesToTicks(targetDegrees, Constants.Turret.kGearRatio, Constants.Turret.kTicksPerRevolution));
  }

  /**
   * Turns the turret at the given output
   * @param percentOutput The output to turn the turret at
   */
  public void turnPercentOut(double percentOutput) {
    turret.set(ControlMode.PercentOutput, percentOutput);
  }

  /**
   * Turns the turret to the given degrees
   * @param ticks The ticks to turn the turret to (absolute)
   */
  public void turnPos(double ticks) {
    turret.set(ControlMode.Position, ticks);
  }

  /**
   * Changes PID constants based on error
   * @param errorDegrees  2 sets of PID constants, 1 for larger errors, 1 for smaller errors
   */
  private void changePIDSlot(double errorDegrees) {
    if(Math.abs(errorDegrees) >= Constants.Turret.kEThreshold)
      turret.selectProfileSlot(1, 0);
    else
      turret.selectProfileSlot(0, 0);
  }

  /**
   * If the turret is not being used or if it is within some margin of error, we can shoot
   * @return if we can shoot
   */
  public boolean canShoot() {
    return state == TurretState.CAN_SHOOT || state == TurretState.DISABLED;
  }

  /**
   * Sets the state of the turret
   * @param state turret's new state
   */
  public void setState(TurretState state) {
    this.state = state;
  }

  /** 
   * Finds the current position in degrees
   * @return  the current position of the turret in degrees
   */
  public double getCurrentPositionDegrees() {
    return MathUtils.ticksToDegrees(
      turret.getSelectedSensorPosition(), 
      Constants.Turret.kGearRatio, 
      Constants.Turret.kTicksPerRevolution
    );
  }

  /**
   * @return The current state of the turret
   */
  public TurretState getTurretState() {
    return state;
  }

  /**
   * Logs data about Turret Subsystem to SmartDashboard
   */
  public void log() {
    SmartDashboard.putNumber("Turret Position (Degrees)", getCurrentPositionDegrees());
    SmartDashboard.putNumber("Horizontal Error", limelight.getHorizontalOffset());
    SmartDashboard.putString("Turret State", state.toString());
    SmartDashboard.putNumber("Turret Output", turret.getMotorOutputPercent());
    SmartDashboard.putNumber("Delta E", deltaE.get());
  }
}
