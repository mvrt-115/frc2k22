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
    TARGETING, CAN_SHOOT, DISABLED, FLIPPING
  }

  private TalonSRX turret;

  private Limelight limelight;

  private double targetDegrees;

  private TurretState state;
  AHRS gyro = new AHRS(SPI.Port.kMXP);

  // direction that the turret spins when randomly searching
  private int direction;
  double gyroInitial = 0;

  private Derivitive deltaE;

  /** Creates a new Turret. */
  public Turret(Limelight limelight) {
    this.limelight = limelight;
    turret = TalonFactory.createTalonSRX(42, false);
    turret.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 1, Constants.kTimeoutMs);

    turret.config_kP(0, Constants.Turret.kP);
    turret.config_kI(0, Constants.Turret.kI);
    turret.config_kD(0, Constants.Turret.kD);

    turret.config_kP(1, Constants.Turret.kPLarge);
    turret.config_kI(1, Constants.Turret.kILarge);
    turret.config_kD(1, Constants.Turret.kDLarge);

    turret.setSelectedSensorPosition(0);
    turret.selectProfileSlot(0, 0);

    targetDegrees = 0;

    direction = 1;
    state = TurretState.DISABLED;

    deltaE = new Derivitive();
  }

  @Override
  public void periodic() {
    // determine if we can shoot if we are within some margin of error
    if(Math.abs(getCurrentPositionDegrees() - targetDegrees) <= 2 && state != TurretState.DISABLED)
      setState(TurretState.CAN_SHOOT);
    else
      setState(TurretState.TARGETING);

    // continue looking for target
    if(state == TurretState.FLIPPING) {
      // targetDegrees += gyroInitial - gyro.getAngle();
      // gyroInitial = gyro.getAngle();
      turn();
      if(Math.abs(getCurrentPositionDegrees()) <= 10)
        setState(TurretState.TARGETING);
    } else if(state != TurretState.DISABLED) 
       target();
    else 
      setMotorOutput(0);

    log();
  }

  public void turn() {
    changePIDSlot(getCurrentPositionDegrees() - targetDegrees);
    turret.set(ControlMode.Position, MathUtils.degreesToTicks(targetDegrees, Constants.Turret.kGearRatio, Constants.Turret.kTicksPerRevolution));
  }

  public void target() {
    if(limelight.targetsFound()) {
      // find target position by using current position and data from limelight
      targetDegrees = getCurrentPositionDegrees() + limelight.getHorizontalOffset();

      if(targetDegrees > Constants.Turret.kMaxAngle) {
        setState(TurretState.FLIPPING);
        targetDegrees = Constants.Turret.kMinAngle + targetDegrees - Constants.Turret.kMaxAngle;
      } else if(targetDegrees < Constants.Turret.kMinAngle) {
        setState(TurretState.FLIPPING);
        targetDegrees = Constants.Turret.kMaxAngle + targetDegrees - Constants.Turret.kMinAngle;
      }

      turn();
    }
  }

  /**
   * Changes direction of the turret if the turret goees outside of the min/max angle
   */
  public void changeDirectionIfNeeded() {
    if(getCurrentPositionDegrees() > Constants.Turret.kMaxAngle || 
      getCurrentPositionDegrees() < Constants.Turret.kMinAngle) {
      direction *= -1;
    }
  }

  /**
   * Changes PID Slot based on error
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
   * @return  if we can shoot
   */
  public boolean canShoot() {
    return state == TurretState.CAN_SHOOT || state == TurretState.DISABLED;
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

  public void setMotorOutput(double output) {
    turret.set(ControlMode.PercentOutput, output);
  }
  
  /**
   * Sets the state of the turret
   * @param inState turret's new state
   */
  public void setState(TurretState inState) {
    state = inState;
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
  }
}
