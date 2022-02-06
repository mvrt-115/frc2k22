// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Derivitive;
import frc.robot.util.Limelight;
import frc.robot.util.MathUtils;
import frc.robot.util.TalonFactory;

public class Turret extends SubsystemBase {

  public static enum TurretState {
    TARGETING, FLIPPING, SEARCHING, CAN_SHOOT, DISABLED
  }

  private TurretState state;

  private TalonFX turret, left, right;

  private Limelight limelight;

  private double targetDegrees;

  private int flipCount = 0;

  // AHRS gyro = new AHRS(SPI.Port.kMXP);

  // double gyroInitial = 0;

  // rate of change of angle
  private Derivitive deltaE;

  /** Creates a new Turret. */
  public Turret(Limelight limelight) {
    this.limelight = limelight;

    turret = TalonFactory.createTalonFX(0, false);
    // left = TalonFactory.createTalonSRX(38, false);
    // right = TalonFactory.createTalonSRX(42, true);

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
    turret.setInverted(true);
  }

  @Override
  public void periodic() {
    // if(state != TurretState.DISABLED)
    //  return;
    log();

    SmartDashboard.putBoolean("flipping", state == TurretState.FLIPPING);
    SmartDashboard.putBoolean("<= 10", Math.abs(getCurrentPositionDegrees()) <= 10);

    // continue looking for target
    if(state == TurretState.FLIPPING) {
      turnToTarget();
      
      if(Math.abs(getCurrentPositionDegrees()) <= 10 || Math.abs(getCurrentPositionDegrees() - targetDegrees) <= 10) {
        setState(TurretState.TARGETING);
        updateTargetDegrees();
      }   
    } else {
      updateTargetDegrees();

      if(state == TurretState.TARGETING || state == TurretState.SEARCHING)
        target();
      else
        turnPercentOut(0);
    }
    
    // determine if we can shoot if we are within some margin of error
    if(Math.abs(limelight.getHorizontalOffset()) <= 1 && state != TurretState.FLIPPING)
      setState(TurretState.CAN_SHOOT);
    else if(state != TurretState.FLIPPING)
      setState(TurretState.TARGETING);
    // left.set(ControlMode.PercentOutput, 0.7);
    // right.set(ControlMode.PercentOutput, -0.7);

    
  }

  public void updateTargetDegrees() {
    if(limelight.targetsFound()) {
      // find target position by using current position and data from limelight
      targetDegrees = getCurrentPositionDegrees() + limelight.getHorizontalOffset();

      // deltaE.update(limelight.getHorizontalOffset());

      if(targetDegrees > Constants.Turret.kMaxAngle + 20) {
        setState(TurretState.FLIPPING);

        System.out.println("entering flip max");

        flipCount++;
        SmartDashboard.putNumber("flip", flipCount);


        targetDegrees = Constants.Turret.kMinAngle + targetDegrees - Constants.Turret.kMaxAngle;
      } else if(targetDegrees < Constants.Turret.kMinAngle - 20) {
        setState(TurretState.FLIPPING);

        targetDegrees = Constants.Turret.kMaxAngle + targetDegrees - Constants.Turret.kMinAngle;

        System.out.println("entering flip min");

        flipCount--;
        SmartDashboard.putNumber("flip", flipCount);
      }
    }
  }

  /**
   * Moves the turret to lock onto the target. If a target has been found, the turret will move there. Otherwise,
   * the turret will turn until it does see something.
   */
  public void target() {
    if(limelight.targetsFound()) {
      state = TurretState.TARGETING;

      turnToTarget();
    } else {
      
    }
  }

  /**
   * Turns to the desired degrees
   */
  private void turnToTarget() {
    // changePIDSlot(getCurrentPositionDegrees() - targetDegrees);
    SmartDashboard.putNumber("Turning to", targetDegrees);
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
    SmartDashboard.putNumber("Target Degrees", targetDegrees);
  }
}
