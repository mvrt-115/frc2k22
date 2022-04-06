// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Limelight;
import frc.robot.util.MathUtils;
import frc.robot.util.RollingAverage;
import frc.robot.util.TalonFactory;
import frc.robot.Constants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

public class Shooter extends SubsystemBase {

  public enum ShooterState
  {
    OFF, SPEEDING, ATSPEED;
  }

  public enum HoodState
  {
    OFF, ADJUSTING, ATPOSITION;
  }

  private final int LEADER_ID = 12;

  private BaseTalon flywheelLeader;

  // Attributes of flywheel
  private ShooterState state;
  private double targetRPM = 0;

  private RollingAverage rpm;
  private Limelight limelight;

  private Turret turret;
  private Drivetrain drivetrain;

  /** Creates a new Shooter. */
  public Shooter(Limelight limelight, Drivetrain drivetrain, Turret turret) {
    // Mind Bending Test aon the Reality of our Situation
    flywheelLeader = TalonFactory.createTalonFX(LEADER_ID, false);
    flywheelLeader.setNeutralMode(NeutralMode.Coast);


    // Sets up PIDF
    flywheelLeader.config_kP(Constants.kPIDIdx, Constants.Flywheel.P);
    flywheelLeader.config_kI(Constants.kPIDIdx, Constants.Flywheel.I);
    flywheelLeader.config_kD(Constants.kPIDIdx, Constants.Flywheel.D);
    flywheelLeader.config_kF(Constants.kPIDIdx, Constants.Flywheel.F);

    this.limelight = limelight;

    state = ShooterState.OFF;
    // hoodState = HoodState.OFF;
    rpm = new RollingAverage(Constants.Flywheel.NUM_AVG);

    this.turret = turret;
    this.drivetrain = drivetrain;
  }

  public void runMotor(double out ){
    flywheelLeader.set(ControlMode.PercentOutput, out);
  } 

  /**
   * Stops Shooter
   */
  public void stopFlywheel() {
    flywheelLeader.set(ControlMode.PercentOutput, 0.3);
  }

  /**
   * Get the current flywheel rpm
   * @return rpm
   */
  public double getCurrentRPM() {
    return rpm.getAverage();
  }

  /**
   * Sets the target rpm of the shooter
   * @param exit_velocity (an rpm value)
   */
  public void setTargetRPM(double exit_velocity) {
    targetRPM = Math.min(Constants.Flywheel.MAX_RPM, exit_velocity);
    
    if(exit_velocity == 0)
      setState(ShooterState.OFF);
    else if(!allWithinRPMError(targetRPM))
      setState(ShooterState.SPEEDING);
    else
      setState(ShooterState.ATSPEED);
  }

  /**
   * Sets the state of the Flywheel
   * Sets target rpm to 0 if state is OFF
   * @param _state
   */
  public void setState(ShooterState _state) {
      this.state = _state;

      if(_state == ShooterState.OFF)
        targetRPM = 0;
  }

  /**
   * Returns the state of the shooter
   * @return the state of the shooter
   */
  public ShooterState getState() {
      return state;
  }

  /**
   * Log Shooter values to SmartDashboard
   */
  public void log()
  {
    // SmartDashboard.putNumber("Output", flywheelLeader.getMotorOutputPercent());
    // SmartDashboard.putNumber("Flywheel RPM", getCurrentRPM());
    // SmartDashboard.putNumber("RPM Needed", getRequiredRPM());
    // SmartDashboard.putString("Shooter State", state.toString());
    // SmartDashboard.putNumber("Target RPM", targetRPM);
    // SmartDashboard.putNumber("Turret Offset", getCalculatedOffset());
    // SmartDashboard.putNumber("Added RPM", getCalculatedAddRPM());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
    // With servos all this is kinda unnecessary but idt it matters so we can keep it

    rpm.updateValue(MathUtils.ticksToRPM(flywheelLeader.getSelectedSensorVelocity(), Constants.Flywheel.TICKS_PER_REVOLUTION, Constants.Flywheel.GEAR_RATIO));
    log();
    
    // Sets state periodically
     switch(state) {
       case OFF:
         stopFlywheel();
         break;
       case SPEEDING:
         flywheelLeader.set(ControlMode.Velocity, MathUtils.rpmToTicks(targetRPM, Constants.Flywheel.TICKS_PER_REVOLUTION, Constants.Flywheel.GEAR_RATIO));
         if(allWithinRPMError(targetRPM)) {
           setState(ShooterState.ATSPEED);
         }
          
         break;
       case ATSPEED:
         // flywheelLeader.set(ControlMode.Velocity, MathUtils.rpmToTicks(targetRPM, Constants.Flywheel.TICKS_PER_REVOLUTION, Constants.Flywheel.GEAR_RATIO));
         if(!allWithinRPMError(targetRPM))
           setState(ShooterState.SPEEDING);
         break;
     }

    // moveAlign();
  }

  public double getStationaryRPM()
  {
    // power series regression from testing data
    // linear constant to slightly tune the shot, -> Limelight distances range from 80 to 220
    // *currently set to 0, I suggest it should be 1*
    double x = limelight.getHorizontalDistance() * Constants.Flywheel.STRETCH_CONSTANT;

    // double rpm = Math.min(333*Math.pow(limelight.getHorizontalDistance() * Constants.Flywheel.STRETCH_CONSTANT, 0.522) + Constants.Flywheel.LIN_CONST * limelight.getHorizontalDistance(), Constants.Flywheel.MAX_RPM);
   
    // double rpm = 479.18*Math.pow(x, 0.4418) + Constants.Flywheel.LIN_CONST * limelight.getHorizontalDistance();
    
    // Use this if power equation is not working
    double rpm = 2040.9 * Math.pow(Math.E, 0.0058 * x);

    // THIS IS NOT FOR SHOOTING ON THE MOVE LOL, JUST FOR DEALING WITH LIMELIGHT ERROR
    if(Math.abs(limelight.getHorizontalOffset())>Constants.Flywheel.ALIGN_ERROR)
    {
      rpm = rpm - Constants.Flywheel.ADJ_HORIZ_ERROR*limelight.getHorizontalOffset();
    }

    rpm+=Constants.Flywheel.REG_CONSTANT;

    return rpm;
  }

  /**
   * Get required RPM for shooter
   * @return required rpm for shooter
   */
  public double getRequiredRPM() {

    double rpm = getStationaryRPM();// + getCalculatedAddRPM();

    return rpm;
  }

  /**
   * Get required hood angle from limelight distance
   * @return Required Hood Angle
   */
  public double getRequiredAng() {
    // metric values will be used until return
    double distance = limelight.getHorizontalDistance();

    // distance from center of hub
    double dx = distance + Units.inchesToMeters(24);

    // function to get theta value
    double angle_proj = 0;

    // Set the angle from 0 to 20 (60 to 80) based on the distance
    if(dx<=2.78) {
      angle_proj = 20 - (5.6845 * Math.pow(dx, 2) - 33.067 * dx + 109.07 - 60);
      
      if(angle_proj>20)
        angle_proj = 20;
      else if(angle_proj<0)
        angle_proj = 0;
    }
    else {
      angle_proj = 20;
    }

    return angle_proj;
  }

  /**
   * @param target -- the target RPM
   * @param acceptableError -- the acceptable +- error range
   * @return boolean whether the RPM is within the acceptable error or not
   */
  private boolean allWithinRPMError(double targetSpeed) {
    return Math.abs(rpm.getAverage() - targetSpeed) <= Constants.Flywheel.ACCEPTABLE_ERROR;
  }

  /**
   * Adjusts the turret to the correct offset
   */
  public void moveAlign()
  {
    turret.setOffset(getCalculatedOffset());
  }

  public double getCalculatedOffset()
  {
    double driveSpeed = (drivetrain.getSpeeds().leftMetersPerSecond+drivetrain.getSpeeds().rightMetersPerSecond)/2;
    double initSpeed = getBaseVelocityFromWheel();
    double diff_const = initSpeed - driveSpeed*Math.cos(Math.toRadians(turret.getCurrentPositionDegrees()+limelight.getHorizontalOffset()));
    double vert_drive_comp = driveSpeed*Math.sin(Math.toRadians(turret.getCurrentPositionDegrees()+limelight.getHorizontalOffset()));
    double totSpeed = Math.sqrt(vert_drive_comp*vert_drive_comp + diff_const*diff_const)+initSpeed;

    double offset = 0;

    if(Math.abs(offset)<=1)
      offset = Math.toDegrees(Math.asin(driveSpeed*Math.sin(Math.toRadians(turret.getCurrentPositionDegrees()+limelight.getHorizontalOffset()))/(totSpeed)));

    return offset;
  }

  public double getCalculatedAddRPM()
  {
    double driveSpeed = (drivetrain.getSpeeds().leftMetersPerSecond+drivetrain.getSpeeds().rightMetersPerSecond)/2;
    double initSpeed = getBaseVelocityFromWheel();
    double diff_const = initSpeed - driveSpeed*Math.cos(Math.toRadians(turret.getCurrentPositionDegrees()+limelight.getHorizontalOffset()));
    double vert_drive_comp = driveSpeed*Math.sin(Math.toRadians(turret.getCurrentPositionDegrees()+limelight.getHorizontalOffset()));
    double addSpeed = Math.sqrt(vert_drive_comp*vert_drive_comp + diff_const*diff_const)-initSpeed;

    return getRPMFromVelocity(addSpeed);
  }

  /**
   * Converts the rpm of the flywheel to linear velocity
   * @return linear velocity
   */
  public double getBaseVelocityFromWheel() {
    return getStationaryRPM()/Constants.Flywheel.RADIUS/60;
  }

  /**
   * Converts the rpm of the flywheel to linear velocity
   * @return linear velocity
   */
  public double getVelocityFromWheelRPM(double rpm) {
    return rpm/Constants.Flywheel.RADIUS/60;
  }

  /**
   * Converts the linear velocity of the flywheel to rpm
   * @return rpm
   */
  public double getRPMFromVelocity(double velocity) {
    return velocity*Constants.Flywheel.RADIUS*60;
  }

  /**
   * @return BaseTalon flywheel leader
   */
  public BaseTalon getMotor()
  {
    return flywheelLeader;
  }
}
