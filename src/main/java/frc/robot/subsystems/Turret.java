
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Limelight;
import frc.robot.util.MathUtils;
import frc.robot.util.TalonFactory;

public class Turret extends SubsystemBase {

  public static enum TurretState {
    TARGETING, FLIPPING, CAN_SHOOT, DISABLED
  }

  public TalonFX turret;
  private Limelight limelight;
  private DigitalInput magLimit;

  private TurretState state;

  private double lastError;
  private double lastTime;
  private double area;

  private double targetDegrees;
  private double searchDirection;
  private boolean searchFlipping;

  private double offset;

  private static Turret turretInstance = new Turret();

  public static Turret getInstance() {
    return turretInstance;
  }

  /** Creates a new Turret. */
  public Turret() {
    turret = TalonFactory.createTalonFX(5, true);
    limelight = Limelight.getInstance();
    magLimit = new DigitalInput(9);

    state = TurretState.DISABLED;

    lastError = 0;
    lastTime = 0;
    area = 0;

    targetDegrees = 0;
    searchDirection = 1;
    searchFlipping = false;

    offset = 0;

    turret.config_kP(0,0.1);
    turret.config_kI(0,Constants.Turret.kI);
    turret.config_kD(0,0);

    resetEncoder();

    turret.selectProfileSlot(0, 0);

    // turret.configForwardSoftLimitEnable(true, 0);
    // turret.configReverseSoftLimitEnable(true, 0);
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
    if(state==TurretState.DISABLED) return;
    visionControl();
    
    //  switch(state) {
    //    case DISABLED:
    //      break;
    //    case FLIPPING:
    //     //  flip();
    //      break;
    //    case CAN_SHOOT:
    //    case TARGETING:
    //     if(Math.abs(targetDegrees - getCurrentPositionDegrees()) < 2 && Math.abs(limelight.getHorizontalOffset()) >= 4) {
    //       visionControl();
    //       updateLastVariables();
    //     } else {
    //       // fieldOrientedControl();
    //       lastError = 0;
    //       area = 0;
    //     }
         
    //      break;
    //  }      
  }

  /**
   * Targets using a pid on the limelight error
   */
  public void visionControl() {
    double error = (limelight.getHorizontalOffset() + (offset)) / 30;
    double time = Timer.getFPGATimestamp();

    area += lastError * (Timer.getFPGATimestamp() - lastTime);

    double output = (Constants.Turret.kP * error) +
      (Constants.Turret.kI * area) + 
      (((error - lastError) / (time - lastTime)) * Constants.Turret.kD);

    SmartDashboard.putNumber("turret output", output);
    SmartDashboard.putNumber("turret error", error);

      if(getCurrentPositionDegrees() < -30 && output < 0)
        output = 0;
      else if(getCurrentPositionDegrees() > 100 && output > 0)
        output = 0;
      

    turret.set(ControlMode.PercentOutput, output);

    // if(Math.abs(limelight.getHorizontalOffset() + offset) > Constants.Turret.kTurretError && limelight.targetsFound()) {
    //   if(output < 0 && getCurrentPositionDegrees() < Constants.Turret.kMinAngle)
    //     output = 0;
    //   else if(output > 0 && getCurrentPositionDegrees() > Constants.Turret.kMaxAngle)
    //     output = 0;

    //   turret.set(ControlMode.PercentOutput, output);
    // } else {
    //   turret.set(ControlMode.PercentOutput, 0);
    //   setState(TurretState.CAN_SHOOT);
    // }
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
   * Searches for a target by going from the min to the max degrees
   */
  public void search() {
    if(searchDirection == 1)
      setPercentOutput(0.5);
    else
      setPercentOutput(-0.5);

    if(!searchFlipping && (turret.getSelectedSensorPosition() >= Constants.Turret.kMaxAngle || 
      turret.getSelectedSensorPosition() <= Constants.Turret.kMinAngle)) {

      searchFlipping = true;
      searchDirection *= -1;
    }

    if(searchFlipping && Math.abs(getCurrentPositionDegrees()) <= 10)
      searchFlipping = false;

    if(limelight.targetsFound())
      setState(TurretState.TARGETING);
  }

  /**
   * Turns the turret to a rough estimate of where the hub is using the gyro and current turret position
   */
  public void fieldOrientedControl() {

    final double hubX = 8.242;
    final double hubY = 4.051;
    
    double robotX = Drivetrain.getInstance().getPose().getX();
    double robotY = Drivetrain.getInstance().getPose().getY();

    double rawAngleDiff = -Math.atan2(hubY - robotY, hubX - robotX);
  
    double turnAngle = rawAngleDiff - Drivetrain.getInstance().getGyroAngle().getRadians();

    targetDegrees = normalizeAngle(Math.toDegrees(turnAngle));

    SmartDashboard.putNumber("targ  ", targetDegrees);

    turnDegrees(targetDegrees);
  }

  /**
   * Updates the lastError and lastTime variables that are used for pid
   */
  private void updateLastVariables() {
    lastError = (limelight.getHorizontalOffset() + offset) / 30;
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
   * Normalizes the angle to fit the bounds [kMinAngle, kMaxAngle]
   * @param degrees
   */
  public double normalizeAngle(double degrees) {

    // degrees = -degrees;

    // if(degrees > Constants.Turret.kMaxAngle)
    //   degrees = Constants.Turret.kMinAngle + (degrees + Constants.Turret.kMinAngle);
    // else if(degrees < Constants.Turret.kMinAngle)
    //   degrees = Constants.Turret.kMaxAngle + (degrees + Constants.Turret.kMaxAngle);

    return Math.max(Constants.Turret.kMinAngle, Math.min(degrees, Constants.Turret.kMaxAngle));
  }

  public void zero() {
    turret.set(ControlMode.Position, 0);
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

  public void setOffset(double offset) {
    this.offset = offset * 0.7;
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

  public double getOffset() {
    return offset;
  }

  public BaseTalon getMotor() {
    return turret;
  }

  public Limelight getLimelight()
  {
    return limelight;
  }

  public boolean targetFound()
  {
    return limelight.targetsFound();
  }

  /**
   * Logs data about Turret Subsystem to SmartDashboard
   */
  public void log() {
    SmartDashboard.putNumber("Turret Position (Degrees)", getCurrentPositionDegrees());
    SmartDashboard.putNumber("Horizontal Error", limelight.getHorizontalOffset()+offset);
    SmartDashboard.putString("Turret State", state.toString());
    SmartDashboard.putNumber("Turret Output", turret.getMotorOutputPercent());
    // SmartDashboard.putNumber("Direction", searchDirection);
    // SmartDashboard.putNumber("Offset", getOffset());
    SmartDashboard.putNumber("Target Degrees", targetDegrees);
  }
}