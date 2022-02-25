// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.Servo;
import frc.robot.util.Limelight;
// import frc.robot.util.LinearActuator;
import frc.robot.util.MathUtils;
import frc.robot.util.RollingAverage;
import frc.robot.util.TalonFactory;
import frc.robot.Constants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

  private final double MAX_RPM = 8000;
  // private final double MAX_ANGLE = 40;

  private final int LEADER_ID = 12;
  // private final int HOOD_ID = 22; //0

  // Linear Actuator IDs (random rn)
  // private final int LEFT_HOOD_ID = 100; // left actuator
  // private final int RIGHT_HOOD_ID = 100; // right actuator

  private BaseTalon flywheelLeader;
  // private BaseTalon hoodMotor;

  // ACtuators
  // private LinearActuator leftActuator;  // left actuator
  // private LinearActuator rightActuator;  // right actuator

  // Attributes of flywheel
  private ShooterState state;
  // private HoodState hoodState;
  private double targetRPM = 0;

  // Target Angle in radians
  // private double targetAng = 0;

  private RollingAverage rpm;
  private Limelight limelight;

  /** Creates a new Shooter. */
  public Shooter(Limelight limelight/*, Drivetrain drivetrain, Turret turret*/) {
    // Mind Bending Test aon the Reality of our Situation
    flywheelLeader = TalonFactory.createTalonFX(LEADER_ID, true);
    // hoodMotor = TalonFactory.createTalonFX(HOOD_ID, false);

    // hoodMotor.setSelectedSensorPosition(0);
    // flywheelFollower.follow(flywheelLeader);


    // Creating Actuators and setting up initial conditions
    // leftActuator = new LinearActuator(new Servo(LEFT_HOOD_ID), 
    //                                   Constants.Hood.HOOD_RADIUS, Constants.Actuator.DIST_FROM_BASE,
    //                                   Constants.Actuator.ACT_HEIGHT, Constants.Actuator.MAX_HEIGHT,
    //                                   Constants.Actuator.DEGREES_FROM_HORIZONTAL);
    // rightActuator = new LinearActuator(new Servo(RIGHT_HOOD_ID), 
    //                                   Constants.Hood.HOOD_RADIUS, Constants.Actuator.DIST_FROM_BASE,
    //                                   Constants.Actuator.ACT_HEIGHT, Constants.Actuator.MAX_HEIGHT,
    //                                   Constants.Actuator.DEGREES_FROM_HORIZONTAL);

    // Initializing them at their min height
    // leftActuator.setPosition(0);
    // rightActuator.setPosition(0);

    //leadActuator.getTalon().setNeutralMode(NeutralMode.Brake);
    //followActuator.getTalon().setNeutralMode(NeutralMode.Brake);


    // Sets up PIDF
    flywheelLeader.config_kP(Constants.kPIDIdx, Constants.Flywheel.P);
    flywheelLeader.config_kI(Constants.kPIDIdx, Constants.Flywheel.I);
    flywheelLeader.config_kD(Constants.kPIDIdx, Constants.Flywheel.D);
    flywheelLeader.config_kF(Constants.kPIDIdx, Constants.Flywheel.F);

    /*hoodMotor.config_kP(Constants.kPIDIdx, Constants.Hood.P);
    hoodMotor.config_kI(Constants.kPIDIdx, Constants.Hood.I);
    hoodMotor.config_kD(Constants.kPIDIdx, Constants.Hood.D);

    adjjustFF = Constants.Hood.kFF * Math.cos(hoodMotor.getSelectedSensorPosition());

    hoodMotor.setNeutralMode(NeutralMode.Brake);*/

    this.limelight = limelight;

    state = ShooterState.OFF;
    // hoodState = HoodState.OFF;
    rpm = new RollingAverage(Constants.Flywheel.NUM_AVG);
  }

  /**
   * Stops Shooter
   */
  public void stopFlywheel() {
    flywheelLeader.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Stops Hood position change
   */
  public void stopHood() {
    // hoodMotor.set(ControlMode.PercentOutput, 0);

    // servos don't need a method like this
  }

  /**
   * Puts hood back to original state
   */
  public void resetHood() {
    // leftActuator.setPosition(0);
  }

  /**
   * Get the current flywheel rpm
   * @return rpm
   */
  public double getCurrentRPM() {
    return rpm.getAverage();
  }

  /**
   * Gets the current hood angle
   * @return angle
   */
  public double getCurrentAngle() {
    return 0;

    // return (leftActuator.getHoodAngle()+rightActuator.getHoodAngle())/2;
  }

  /**
   * Sets the target rpm of the shooter
   * @param exit_velocity (an rpm value)
   */
  public void setTargetRPM(double exit_velocity) {
    targetRPM = Math.min(MAX_RPM, exit_velocity);
    
    if(exit_velocity == 0)
      setState(ShooterState.OFF);
    else if(!allWithinRPMError(targetRPM))
      setState(ShooterState.SPEEDING);
    else
      setState(ShooterState.ATSPEED);
  }

  // /**
  //  * Set shooter hood angle
  //  * @param angle (in degrees)
  //  */
  // public void setTargetAngle(double angle) {
  //   targetAng = Math.min(angle, MAX_ANGLE);
  //   if (angle == 0)
  //     setHoodState(HoodState.OFF);
  //   else
  //     setHoodState(HoodState.ADJUSTING);
  // }

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

  // /**
  //  * Sets the state of the Hood
  //  * Sets the target angle to 0 if state is OFF
  //  * @param _state
  //  */
  // public void setHoodState(HoodState _state) {
  //   this.hoodState = _state;

  //   if (_state == HoodState.OFF)
  //     targetAng = 0;
  // }

  /**
   * Log Shooter values to SmartDashboard
   */
  public void log()
  {
    SmartDashboard.putNumber("Flywheel RPM", getCurrentRPM());
    SmartDashboard.putNumber("RPM Needed", getRequiredRPM());
    SmartDashboard.putString("Shooter State", state.toString());
    SmartDashboard.putNumber("Target RPM", targetRPM);
    // SmartDashboard.putNumber("Hood Angle", getCurrentAngle());
    // SmartDashboard.putNumber("Hood Angle Ticks", hoodMotor.getSelectedSensorPosition());
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

    /*switch(hoodState) {
      case OFF:
        stopHood();
        break;
      case ADJUSTING:
        // hoodMotor.set(ControlMode.Position, degreesToTicks(targetAng));

        // leftActuator.setPositionFromAngle(targetAng);
        // rightActuator.setPositionFromAngle(targetAng);

        if(allWithinPositionError(targetAng))
        {
          setHoodState(HoodState.ATPOSITION);
        }
        break;
      case ATPOSITION:
        if(!allWithinPositionError(targetRPM))
        {
          setHoodState(HoodState.ADJUSTING);
        }
        break;
    }*/

    // Adjusts turret to correct offset
    // moveAlign();
  }

  /**
   * Get required RPM for shooter
   * @return required rpm for shooter
   */
  public double getRequiredRPM() {
    /*
    // metric values will be used until return
    double distance = Units.inchesToMeters(limelight.getHorizontalDistance());

    // distance from center of hub
    double dx = distance + Units.inchesToMeters(24);

    // function to get velocity value given shooter height is 3ft. from the ground
    double vel_proj = 0;

    if(dx<=1.72)
      vel_proj = -12.186 * Math.pow(dx, 3) + 54.736 * Math.pow(dx, 2) - 81.631 * dx + 48.166;
    else
      vel_proj = 0.2546 * Math.pow(dx, 2) - 0.0295 * dx + 7.0226;
    
    return 1.8*(Units.metersToInches(60) * vel_proj / (Constants.Flywheel.RADIUS * 2 * Math.PI));
    */

    return 640.55*Math.pow(limelight.getHorizontalDistance(), 0.3468);
    // double dist = limelight.getHorizontalDistance();
    // return 1.05*(-0.035*Math.pow(dist, 2) + 19.121*dist + 1591.2);
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

  /*private boolean allWithinPositionError(double targetAngle) {
    if (hoodMotor != null)
    {
      return Math.abs(getCurrentAngle() - targetAngle) <= Constants.Hood.ACCEPTABLE_ERROR;
    }

    if (leftActuator != null)
    {
        //return Math.abs(leftActuator.getHoodAngle() - targetAngle) <= Constants.Hood.ACCEPTABLE_ERROR;

        return true; // we aren't able to test the hood rn so have this always return true
    }

    return true;
  }*/

  /**
   * Adjusts the turret to the correct offset
   */
  /*public void moveAlign()
  {
    double driveSpeed = (drivetrain.getSpeeds().leftMetersPerSecond+drivetrain.getSpeeds().rightMetersPerSecond)/2;
    
    double initSpeed = getVelocityFromWheelRPM();
    double addSpeed = Math.sqrt(Math.pow(initSpeed, 2) + 2*initSpeed*driveSpeed*Math.cos(1-turret.getCurrentPositionDegrees())+Math.pow(driveSpeed, 2));

    double offset = Math.asin(driveSpeed*Math.sin(1-turret.getCurrentPositionDegrees())/(initSpeed+addSpeed));

    turret.setOffset(offset);
  }*/

  /**
   * Converts the rpm of the flywheel to linear velocity
   * @return linear velocity
   */
  public double getVelocityFromWheelRPM() {
    return getRequiredRPM()/Constants.Flywheel.RADIUS/60;
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
