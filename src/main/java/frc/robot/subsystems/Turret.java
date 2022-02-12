// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Limelight;
import frc.robot.util.MathUtils;
import frc.robot.util.TalonFactory;

// TODO: Edge cases for turret (need to recalibrate should be able to recalibrate no matter where turret is)

public class Turret extends SubsystemBase {

  public static enum TurretState {
    TARGETING, FLIPPING, SEARCHING, CAN_SHOOT, DISABLED
  }

  private TurretState state;

  private TalonFX turret;
  private Limelight limelight;
  private DigitalInput magLimit;

  private double targetDegrees;
  private double searchDirection;
  private boolean zeroing;

  /** Creates a new Turret. */
  public Turret(Limelight limelight) {
    this.limelight = limelight;

    turret = TalonFactory.createTalonFX(0, false);
    magLimit = new DigitalInput(0);

    targetDegrees = 0;
    searchDirection = 1;
    zeroing = false;

    state = TurretState.DISABLED;

    turret.config_kP(0, Constants.Turret.kP);
    turret.config_kI(0, Constants.Turret.kI);
    turret.config_kD(0, Constants.Turret.kD);

    resetEncoder();
    turret.selectProfileSlot(0, 0);
    turret.setInverted(true);
  }

  @Override
  public void periodic() {
    log();
    if(state != TurretState.FLIPPING)
      updateTargetDegrees();

    
    // continue looking for target
    if(state == TurretState.FLIPPING) {
      turnDegrees(targetDegrees);
      
      if(magLimit.get()) {
        setState(TurretState.TARGETING);
        updateTargetDegrees();
      }
        
    }
    if(state == TurretState.TARGETING) {
      target();
    } else if(state != TurretState.FLIPPING){ 
      if(limelight.targetsFound())
        setPercentOutput(0);
    }

    if(state == TurretState.SEARCHING) {
      double pos = getCurrentPositionDegrees();
      if(Math.abs(pos - Constants.Turret.kMinAngle) < Constants.Turret.kEThreshold) 
        zeroing = true;
      if(Math.abs(pos) < Constants.Turret.kEThreshold)
        zeroing = false;
      else if(Math.abs(Math.abs(pos) - Constants.Turret.kMinAngle) < Constants.Turret.kEThreshold) 
        searchDirection *= -1;

      if(zeroing) 
        turnDegrees(0);
      else
        setPercentOutput(searchDirection * Constants.Turret.kTurnSpeed);
    }
    
    // determine if we can shoot if we are within some margin of error
    if(Math.abs(limelight.getHorizontalOffset()) <= 1 && state != TurretState.FLIPPING)
      setState(TurretState.CAN_SHOOT);
    else if(state != TurretState.FLIPPING)
      setState(TurretState.TARGETING);
  }

  public void updateTargetDegrees() {
    if(limelight.targetsFound() && Math.abs(limelight.getHorizontalOffset()) > 4) {
      // find target position by using current position and data from limelight
      targetDegrees = getCurrentPositionDegrees() + limelight.getHorizontalOffset();

      if(targetDegrees > Constants.Turret.kMaxAngle + 20) {
        setState(TurretState.FLIPPING);

        targetDegrees = Constants.Turret.kMinAngle + targetDegrees - Constants.Turret.kMaxAngle;
      } else if(targetDegrees < Constants.Turret.kMinAngle - 20) {
        setState(TurretState.FLIPPING);

        targetDegrees = Constants.Turret.kMaxAngle + targetDegrees - Constants.Turret.kMinAngle;
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

      turnDegrees(targetDegrees);
    } else {
      state = TurretState.SEARCHING;
    }
  }

  /**
   * Turns to the desired degrees
   * @param degrees the desired degrees
   */
  private void turnDegrees(double degrees) {
    turret.set(
      ControlMode.Position,
      MathUtils.degreesToTicks(
        degrees, 
        Constants.Turret.kGearRatio, 
        Constants.Turret.kTicksPerRevolution
      )
    );
  }

  /**
   * Turns the turret at the given output
   * @param percentOutput The output to turn the turret at
   */
  public void setPercentOutput(double percentOutput) {
    turret.set(ControlMode.PercentOutput, percentOutput);
  }

  /**
   * Resets the encoder position to 0
   */
  public void resetEncoder() {
    turret.setSelectedSensorPosition(0);
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
   * @return The current state of the turret
   */
  public TurretState getTurretState() {
    return state;
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
   * Gets if the mag limit switch is aligned
   * @return The alignment state (true/false)
   */
  public boolean getMagAligned() {
    return !magLimit.get();
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
    SmartDashboard.putBoolean("Is Aligned", getMagAligned());
  }
}
