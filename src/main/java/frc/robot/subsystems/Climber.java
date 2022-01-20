// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  public TalonFX pivot; //motor for both pivoting arms
  public TalonFX leftTelescopic, rightTelescopic; // motors for each telescopic arm, controlling extending and collapsing motions
  public DigitalInput leftPivotProximity, rightPivotProximity, leftTelescopicProximity, rightTelescopicProximity; //limit switches 
    //for detecting whether robot is hooked on rungs or not for each type of arm
  public AnalogPotentiometer potentiometer; //potentiometer to measure the turn of the pivoting arm

  /**
   * Initializes all objects and reconfigures all motors to requirements
   */
  public Climber() {
    //initializes all motors and sensors
    pivot = new TalonFX(Constants.Climber.pivotID);
    leftTelescopic = new TalonFX(Constants.Climber.leftTelescopicID);
    rightTelescopic = new TalonFX(Constants.Climber.rightTelescopicID);
    
    leftPivotProximity = new DigitalInput(Constants.Climber.leftPivotProximityChannel);
    rightPivotProximity = new DigitalInput(Constants.Climber.rightPivotProximityChannel);
    leftTelescopicProximity = new DigitalInput(Constants.Climber.leftTelescopicProximityChannel);
    rightTelescopicProximity = new DigitalInput(Constants.Climber.rightTelescopicProximityChannel);

    potentiometer = new AnalogPotentiometer(Constants.Climber.potentiometerChannel);

    //reconfiguring all motors
    pivot.configFactoryDefault();
    leftTelescopic.configFactoryDefault();
    rightTelescopic.configFactoryDefault();
  

    pivot.setInverted(TalonFXInvertType.Clockwise);
    leftTelescopic.setInverted(false);
    rightTelescopic.setInverted(false);
    
    rightTelescopic.follow(leftTelescopic);
    pivot.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx, 
      Constants.kTimeoutMs);
    

    leftTelescopic.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx,
      Constants.kTimeoutMs);

    pivot.config_kP(Constants.kPIDIdx, Constants.Climber.pivotkP);
    pivot.config_kI(Constants.kPIDIdx, Constants.Climber.pivotkI);
    pivot.config_kD(Constants.kPIDIdx, Constants.Climber.pivotkD);
    pivot.config_kF(Constants.kPIDIdx, Constants.Climber.pivotkF);

    leftTelescopic.config_kP(Constants.kPIDIdx, Constants.Climber.telekP);
    leftTelescopic.config_kI(Constants.kPIDIdx, Constants.Climber.telekI);
    leftTelescopic.config_kD(Constants.kPIDIdx, Constants.Climber.telekD);
    leftTelescopic.config_kF(Constants.kPIDIdx, Constants.Climber.telekF);

    rightTelescopic.config_kP(Constants.kPIDIdx, Constants.Climber.telekP);
    rightTelescopic.config_kI(Constants.kPIDIdx, Constants.Climber.telekI);
    rightTelescopic.config_kD(Constants.kPIDIdx, Constants.Climber.telekD);
    rightTelescopic.config_kF(Constants.kPIDIdx, Constants.Climber.telekF);
  }

  /**
   * Sets speed to given motor
   * @param motor the motor that needs to be run
   * @param speed speed at which the motor needs to be run
  */
  public void setSpeed(BaseTalon motor, double speed){
    motor.set(ControlMode.PercentOutput, speed); 
  }

  /**
   * Stops motors
   * @param motor the motor that needs to be stopped
  */
  public void stopMotor(BaseTalon motor){
    setSpeed(motor, 0);
  }

  /**
    * Sets position of the given motor
    * @param motor the motor that needs to be set 
    * @param finalPosition final position of the motor
   */
  public void setPosition(BaseTalon motor, double finalPosition){
    motor.set(ControlMode.Position, finalPosition);
  }

  /**
   * Get the encoder value of a motor
   * @param motor the motor to get the value of
   * @return the number of ticks motor has rotated
   */
  public double getEncoderValue(BaseTalon motor){
    return motor.getSelectedSensorPosition();
  }

  public double getTelescopicPosition()
  {
    return 0;
  }

  public double getPivotAngle()
  {
    return 0;
  }

  /**
   * Get the state of the limit switch
   * @param limitSwitch limit switch to get the state of
   * @return the state of limit switch (true/false)
   */
  public boolean getProximity(DigitalInput proximity){
    return proximity.get();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

} 
