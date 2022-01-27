// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  public TalonFX leftPivot, rightPivot; //motor for both pivoting arms
  public TalonFX telescopic; // motors for each telescopic arm, controlling extending and collapsing motions
  public DigitalInput leftPivotProximity, rightPivotProximity, leftTelescopicProximity, rightTelescopicProximity; //inductive proximity sensors 
    //for detecting whether robot is hooked on rungs or not for each type of arm
  public DigitalInput leftPivotLimit, rightPivotLimit, leftTelescopicLimit, rightTelescopicLimit; //inductive proximity sensors 
    //for detecting whether robot is hooked on rungs or not for each type of arm
  public AnalogPotentiometer potentiometerPivot, potentiometerTelescopic; //potentiometer to measure the turn of the pivoting arm

  /**
   * Initializes all objects and reconfigures all motors to requirements
   */
  public Climber() {
    //initializes all motors and sensors
    leftPivot = new TalonFX(Constants.Climber.leftPivotID);
    rightPivot = new TalonFX(Constants.Climber.rightPivotID);
    telescopic = new TalonFX(Constants.Climber.telescopicID);
    
    leftPivotProximity = new DigitalInput(Constants.Climber.leftPivotProximityChannel);
    rightPivotProximity = new DigitalInput(Constants.Climber.rightPivotProximityChannel);
    leftTelescopicProximity = new DigitalInput(Constants.Climber.leftTelescopicProximityChannel);
    rightTelescopicProximity = new DigitalInput(Constants.Climber.rightTelescopicProximityChannel);

    potentiometerPivot = new AnalogPotentiometer(Constants.Climber.potentiometerPivotChannel);
    potentiometerTelescopic = new AnalogPotentiometer(Constants.Climber.potentiometerTelescopicChannel);
    
    //reconfiguring all motors
    leftPivot.configFactoryDefault();
    rightPivot.configFactoryDefault();
    telescopic.configFactoryDefault();
  

    leftPivot.setInverted(TalonFXInvertType.Clockwise);
    rightPivot.setInverted(false);
    telescopic.setInverted(false);
    
    rightPivot.follow(leftPivot);
    
    telescopic.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx, 
      Constants.kTimeoutMs);
    leftPivot.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx,
      Constants.kTimeoutMs);
    rightPivot.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx,
      Constants.kTimeoutMs);

    leftPivot.config_kP(Constants.kPIDIdx, Constants.Climber.pivotkP);
    leftPivot.config_kI(Constants.kPIDIdx, Constants.Climber.pivotkI);
    leftPivot.config_kD(Constants.kPIDIdx, Constants.Climber.pivotkD);
    leftPivot.config_kF(Constants.kPIDIdx, Constants.Climber.pivotkF);

    telescopic.config_kP(Constants.kPIDIdx, Constants.Climber.telekP);
    telescopic.config_kI(Constants.kPIDIdx, Constants.Climber.telekI);
    telescopic.config_kD(Constants.kPIDIdx, Constants.Climber.telekD);
    telescopic.config_kF(Constants.kPIDIdx, Constants.Climber.telekF);

    rightPivot.config_kP(Constants.kPIDIdx, Constants.Climber.telekP);
    rightPivot.config_kI(Constants.kPIDIdx, Constants.Climber.telekI);
    rightPivot.config_kD(Constants.kPIDIdx, Constants.Climber.telekD);
    rightPivot.config_kF(Constants.kPIDIdx, Constants.Climber.telekF);
  }

  /**
   * Sets speed to given motor
   * @param motor the motor that needs to be run
   * @param speed speed at which the motor needs to be run
  */
  public void setSpeed(TalonFX motor, double speed){
    motor.set(ControlMode.PercentOutput, speed); 
  }

  /**
   * Stops motors
   * @param motor the motor that needs to be stopped
  */
  public void stopMotor(TalonFX motor){
    setSpeed(motor, 0);
  }

  /**
    * Sets position of the given motor
    * @param motor the motor that needs to be set 
    * @param finalPosition final position of the motor
   */
  public void setPosition(TalonFX motor, double finalPosition){
    motor.set(ControlMode.Position, finalPosition);
  }

  /**
   * Get the encoder value of a motor
   * @param motor the motor to get the value of
   * @return the number of ticks motor has rotated
   */
  public double getEncoderValue(TalonFX motor){
    return motor.getSelectedSensorPosition();
  }

  public double getTelescopicPosition()
  {
    return 0;
  }

  /**
   * Calculates the angle the pivot has moved through for the selected potentiometer
   * @return angle travelled by pivot
   */
  public double getPivotAnglePoten()
  {
    return 0;
  }

  public double getPivotAngle()
  {
    return 0;
  }

  /**
   * Get the state of the inductive proximity sensor on hook
   * @param limitSwitch proximity seonsor to get the state of
   * @return the state of proximity sensor (true/false)
   */
  public boolean getProximity(DigitalInput proximity){
    return proximity.get();
  }

  /**
   * Get the state of the limit switch
   * @param limitSwitch limit switch to get the state of
   * @return the state of limit switch (true/false)
   */
  public boolean getLimitSwitch(DigitalInput limitSwitch){
    return limitSwitch.get();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

} 
