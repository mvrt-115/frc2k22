
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Limelight;
import frc.robot.util.MathUtils;
import frc.robot.util.TalonFactory;

// TODO: Edge cases for turret (need to recalibrate should be able to recalibrate no matter where turret is)

public class Turret extends SubsystemBase {

  public static enum TurretState {
    TARGETING, FLIPPING, CAN_SHOOT, DISABLED
  }

  private TalonFX turret;
  private Limelight limelight;
  private DigitalInput magLimit;
  private boolean atEdgeRight = false;
  private boolean atEdgeLeft = false;

  private TurretState state;

  private double lastError;
  private double lastTime;
  private double area;

  private double targetDegrees;
  /** Creates a new Turret. */
  public Turret(Limelight limelight) {
    turret = TalonFactory.createTalonFX(5, true);
    this.limelight = limelight;
    magLimit = new DigitalInput(9);

    state = TurretState.DISABLED;

    lastError = 0;
    lastTime = 0;
    area = 0;

    targetDegrees = 0;

    turret.config_kP(0, 0.05);
    turret.config_kI(0, 0);
    turret.config_kD(0, 0.1);

    resetEncoder();

    turret.selectProfileSlot(0, 0);
  }

  @Override
  public void periodic() {
    log();

    // if(getCurrentPositionDegrees() >= Constants.Turret.kMaxAngle) {
    //   targetDegrees = Constants.Turret.kMinAngle + 20;
    //   setState(TurretState.FLIPPING);
    // } else if(getCurrentPositionDegrees() <= Constants.Turret.kMinAngle) {
    //   targetDegrees = Constants.Turret.kMaxAngle - 20;
    //   setState(TurretState.FLIPPING);
    // }

    // if(DriverStation)
        
     switch(state) {
       case DISABLED:
         turret.set(ControlMode.PercentOutput, 0);
         return;
       case FLIPPING:
         flip();
         break;
       case CAN_SHOOT:
       case TARGETING:
         target();
         break;
        default:
        break;
     }

     SmartDashboard.putBoolean("Left Edge Turret", atEdgeLeft);
     SmartDashboard.putBoolean("Right Edge Turret", atEdgeRight);
     
     updateLastVariables();
  }

  /**
   * Targets using a pid on the limelight error
   */
  public void target() {
    double error = (limelight.getHorizontalOffset()) / 30;
    double time = Timer.getFPGATimestamp();

    area += lastError * (Timer.getFPGATimestamp() - lastTime);

    double output = (Constants.Turret.kP * error) +
      (Constants.Turret.kI * area) + 
      (((error - lastError) / (time - lastTime)) * Constants.Turret.kD);

    if(Math.abs((limelight.getHorizontalOffset())) > 7 && limelight.targetsFound()) {
      if(output < 0 && getCurrentPositionDegrees() < Constants.Turret.kMinAngle) {
        output = 0;
        atEdgeRight = true;
      }       
      else if(output > 0 && getCurrentPositionDegrees() > Constants.Turret.kMaxAngle) {
         output = 0;
         atEdgeLeft = true;
      } else {
        atEdgeRight = false;
        atEdgeLeft = false;
        output = Math.min(1, Math.max(-1, output));
      }
      setPercentOutput(output);
    } else {
      setPercentOutput(0);
    }
      
    
  }

  /**
   * Checks for if the turret should go back to targeting
   */
  public void flip() {
    if(Math.abs(getCurrentPositionDegrees() - targetDegrees) < 5) {
      setState(TurretState.TARGETING);
      turret.set(ControlMode.PercentOutput, 0);
    } else {
      turnDegrees(targetDegrees);
    }
  }

  /**
   * Updates the lastError and lastTime variables that are used for pid
   */
  private void updateLastVariables() {
    lastError = (limelight.getHorizontalOffset()) / 30;
    lastTime = Timer.getFPGATimestamp();
  }

  /**
   * Turns to the desired degrees
   * @param degrees the desired degrees
   */
  public void turnDegrees(double degrees) {
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
   * Gets if the mag limit switch is aligned
   * @return The alignment state (true/false)
   */
  public boolean getMagAligned() {
    return !magLimit.get();
  }

  public BaseTalon getMotor() {
    return turret;
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

  public void zero() {
    turret.set(ControlMode.Position, 0);
  }
}