
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
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
  private DigitalInput magLimit;

  private double targetDegrees;
  private double searchDirection;
  private double offset;
  // private boolean zeroing;

  /** Creates a new Turret. */
  public Turret(Limelight limelight) {
    this.limelight = limelight;

    turret = TalonFactory.createTalonFX(5, true);
    magLimit = new DigitalInput(9);

    targetDegrees = 0;
    searchDirection = 1;
    // zeroing = false;

    offset = 0;

    state = TurretState.DISABLED;

    turret.config_kP(0, 0.1);//Constants.Turret.kPLarge);
    turret.config_kI(0, 0);//Constants.Turret.kI);
    turret.config_kD(0, 0);//Constants.Turret.kD);
    resetEncoder();
    turret.selectProfileSlot(0, 0);
  }

  @Override
  public void periodic() {
    log();

    // turret.set(ControlMode.Position, MathUtils.degreesToTicks(180, 2048, Constants.Turret.kGearRatio));
    // SmartDashboard.putNumber("turret ticks", turret.getSelectedSensorPosition(0));

    // unless flipping keep track of the target degrees
    // if(state != TurretState.FLIPPING)
    //   updateTargetDegrees();

    // if(state == TurretState.FLIPPING) {
    //   turnDegrees(targetDegrees);
    // targetDegrees = getCurrentPositionDegrees()
      

      // turret.set(ControlMode.Position, MathUtils.degreesToTicks(targetDegrees, Constants.Turret.kTicksPerRevolution, Constants.Turret.kGearRatio));

      double error = (limelight.getHorizontalOffset()+offset) / 30;
      double time = Timer.getFPGATimestamp();

      area+= lastError * (Timer.getFPGATimestamp() - lastTime);

      double output = (Constants.Turret.kP * error)
      + (Constants.Turret.kI * area) + 
      (((error - lastError) / (time - lastTime)) * Constants.Turret.kD);

      if(Math.abs((limelight.getHorizontalOffset()+offset)) > 1 && limelight.targetsFound()){

        if(output < 0 && getCurrentPositionDegrees() < Constants.Turret.kMinAngle)
          output = 0;
        else if(output > 0 && getCurrentPositionDegrees() > Constants.Turret.kMaxAngle)
          output = 0;
        turret.set(ControlMode.PercentOutput,output);
      }
        
      else
        turret.set(ControlMode.PercentOutput, 0);
      
      lastError = error;
      lastTime = time;

      if(Math.abs(getCurrentPositionDegrees()) >= Constants.Turret.kMaxAngle + 20) {
        // turret.set(ControlMode.PercentOutput, 0);
        setState(TurretState.FLIPPING);
        if(getCurrentPositionDegrees() < 0)
          targetDegrees = 180;
        else
          targetDegrees = -180;
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
    // if(Math.abs(limelight.getHorizontalOffset()+offset) <= 5 && state != TurretState.FLIPPING)
    //   setState(TurretState.CAN_SHOOT);
    // else if(state != TurretState.FLIPPING)
    //   setState(TurretState.TARGETING);
  }

  public void updateTargetDegrees() {
    if(limelight.targetsFound()) { // && Math.abs(limelight.getHorizontalOffset()+offset) > 2) {
      // find target position by using current position and data from limelight
      targetDegrees = getCurrentPositionDegrees() + 1.25 * (limelight.getHorizontalOffset()+offset);

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
    return !magLimit.get();
  }

  public void setOffset(double _offset)
  {
    offset = _offset;
  }

  public double getOffset()
  {
    return offset;
  }

  /**
   * Logs data about Turret Subsystem to SmartDashboard
   */
  public void log() {
    SmartDashboard.putNumber("Turret Position (Degrees)", getCurrentPositionDegrees());
    SmartDashboard.putNumber("Horizontal Error", limelight.getHorizontalOffset()+offset);
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