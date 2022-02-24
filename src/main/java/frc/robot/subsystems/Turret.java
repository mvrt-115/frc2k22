// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.DigitalInput;
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
  private double lastError = 0;
  private double lastTime = 0;

  private TalonFX turret;
  private Limelight limelight;
  private double area = 0;
  // private DigitalInput magLimit;

  private double targetDegrees;
  private double searchDirection;
  // private boolean zeroing;

  /** Creates a new Turret. */
  public Turret(Limelight limelight) {
    this.limelight = limelight;

    turret = TalonFactory.createTalonFX(0, true);
    // magLimit = new DigitalInput(0);

    targetDegrees = 0;
    searchDirection = 1;
    // zeroing = false;

    state = TurretState.DISABLED;

    turret.config_kP(0, Constants.Turret.kP);
    turret.config_kI(0, Constants.Turret.kI);
    turret.config_kD(0, Constants.Turret.kD);

    SmartDashboard.putNumber("turret p", Constants.Turret.kP);
    SmartDashboard.putNumber("turret i", Constants.Turret.kI);
    SmartDashboard.putNumber("turret d", Constants.Turret.kD);

    resetEncoder();
    turret.selectProfileSlot(0, 0);
  }

  @Override
  public void periodic() {
    log();

    // unless flipping keep track of the target degrees
    // if(state != TurretState.FLIPPING)
    //   updateTargetDegrees();

    // if(state == TurretState.FLIPPING) {
    //   turnDegrees(targetDegrees);

    double p = SmartDashboard.getNumber("turret p", Constants.Turret.kP);
    double i = SmartDashboard.getNumber("turret i", Constants.Turret.kI);
    double d = SmartDashboard.getNumber("turret d", Constants.Turret.kD);
    targetDegrees = getCurrentPositionDegrees() + limelight.getHorizontalOffset();

    if(state == TurretState.FLIPPING) {
      turret.set(ControlMode.Position, MathUtils.degreesToTicks(targetDegrees, Constants.Turret.kTicksPerRevolution, Constants.Turret.kGearRatio));
      
      if(Math.abs(targetDegrees - getCurrentPositionDegrees()) < 0) 
        setState(TurretState.TARGETING);
    } else {
      

      // turret.set(ControlMode.Position, MathUtils.degreesToTicks(targetDegrees, Constants.Turret.kTicksPerRevolution, Constants.Turret.kGearRatio));

      area+= lastError * (Timer.getFPGATimestamp() - lastTime);

      turret.set(ControlMode.PercentOutput, 
        (p * limelight.getHorizontalOffset())
        + (i * area) + 
        (((limelight.getHorizontalOffset() - lastError) / (Timer.getFPGATimestamp() - lastTime)) * d)
        );
      
      lastError = limelight.getHorizontalOffset();
      lastTime = Timer.getFPGATimestamp();

      if(Math.abs(getCurrentPositionDegrees()) >= Constants.Turret.kMaxAngle) {
        turret.set(ControlMode.PercentOutput, 0);
        setState(TurretState.FLIPPING);
        if(getCurrentPositionDegrees() < 0)
          targetDegrees = 160;
        else
          targetDegrees = -160;
      }
  }
      
      

    lastTime = Timer.getFPGATimestamp();
      
      // once flipped to a point when target not in view, begin targetting again
    //   if(!limelight.targetsFound()) {
    //     setState(TurretState.TARGETING);
    //     updateTargetDegrees();
    //   }
        
    // }
    // if(state == TurretState.TARGETING) {
    //   target();
    // } else if(state != TurretState.FLIPPING){ 
    //   if(limelight.targetsFound())
    //     setPercentOutput(0);
    // }

    // // if(state == TurretState.SEARCHING) {
    // //   double pos = getCurrentPositionDegrees();
    // //   // // if(Math.abs(pos - Constants.Turret.kMa) < Constants.Turret.kEThreshold) 
    // //   // //   zeroing = false;//true;
    // //   // // if(Math.abs(pos) < Constants.Turret.kEThreshold)
    // //   // //   zeroing = false;
    // //   // if(Math.abs(pos) >= Constants.Turret.kMaxAngle) 
    // //   //   searchDirection *= -1;

    // //   // // if(zeroing) 
    // //   // //   turnDegrees(0);
    // //   // // else
    // //   if(Math.abs(pos - Constants.Turret.kMinAngle) < Constants.Turret.kEThreshold) 
    // //     zeroing = true;
    // //   if(Math.abs(pos) < Constants.Turret.kEThreshold)
    // //     zeroing = false;
    // //   else if(Math.abs(Math.abs(pos) - Constants.Turret.kMinAngle) < Constants.Turret.kEThreshold) 
    // //     searchDirection *= -1;

    // //   if(zeroing) 
    // //     turnDegrees(0);
    // //   else
    // //     setPercentOutput(searchDirection * Constants.Turret.kTurnSpeed);
    // // }
    
    // // determine if we can shoot if we are within some margin of error
    // if(Math.abs(limelight.getHorizontalOffset()) <= 5 && state != TurretState.FLIPPING)
    //   setState(TurretState.CAN_SHOOT);
    // else if(state != TurretState.FLIPPING)
    //   setState(TurretState.TARGETING);
  }

  public void updateTargetDegrees() {
    if(limelight.targetsFound()) { // && Math.abs(limelight.getHorizontalOffset()) > 2) {
      // find target position by using current position and data from limelight
      targetDegrees = getCurrentPositionDegrees() + 1.25 * limelight.getHorizontalOffset();

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
      // state = TurretState.SEARCHING;
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
    return false;//!magLimit.get();
  }

  /**
   * Logs data about Turret Subsystem to SmartDashboard
   */
  public void log() {
    SmartDashboard.putNumber("Turret Position (Degrees)", getCurrentPositionDegrees());
    SmartDashboard.putNumber("Horizontal Error", limelight.getHorizontalOffset());
    SmartDashboard.putString("Turret State", state.toString());
    SmartDashboard.putNumber("Turret Output", turret.getMotorOutputPercent());
    SmartDashboard.putNumber("Direction", searchDirection);
    SmartDashboard.putNumber("Target Degrees", targetDegrees);
    // SmartDashboard.putBoolean("Is Aligned", getMagAligned());
  }

  public BaseTalon getMotor() {
    return turret;
  }
}