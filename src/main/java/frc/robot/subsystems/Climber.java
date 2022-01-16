// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  public BaseTalon pivot; //motor for both pivoting arms
  public BaseTalon leftTelescopic, rightTelescopic; // motors for each telescopic arm, controlling extending and collapsing motions
  public WPI_PigeonIMU gyro; //gyroscopic sensor for detecting angle moved through on the pivoting arms
  public DigitalInput pivotLimitSwitch, teleLimitSwitch; //limit switches for detecting whether robot is hooked on rungs or not for each type of arm

  /**
   * Initializes all objects and reconfigures all motors to requirements
   */
  public Climber() {
    //initializes all motors and sensors
    pivot = new TalonFX(Constants.Climber.pivotID);
    leftTelescopic = new TalonFX(Constants.Climber.leftTelescopicID);
    rightTelescopic = new TalonFX(Constants.Climber.rightTelescopicID);
    
    pivotLimitSwitch = new DigitalInput(Constants.Climber.pivotLimitSwitchID);
    teleLimitSwitch = new DigitalInput(Constants.Climber.teleLimitSwitchID);

    gyro = new WPI_PigeonIMU(Constants.Climber.gyroID);

    //reconfiguring all motors
    pivot.configFactoryDefault();
    leftTelescopic.configFactoryDefault();
    rightTelescopic.configFactoryDefault();
    gyro.configFactoryDefault();
  

    pivot.setInverted(false);
    leftTelescopic.setInverted(false);
    rightTelescopic.setInverted(false);
    
    rightTelescopic.follow(leftTelescopic);
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

  /**
   * Resets gyroscopic sensor
   */
  public void resetGyro(){
    gyro.setCompassAngle(0);
  } 

  /**
   * Get the angle of the gyroscopic sensor on the IMU
   * @return angle of the gyroscopic sensor
   */
  public double getGyroAngle(){
    return gyro.getAngle();
    /*double[] angles = new double[3];
    gyro.getAccelerometerAngles(angles);
    return angles;*/
  }

  /**
   * Get the state of the limit switch
   * @param limitSwitch limit switch to get the state of
   * @return the state of limit switch (true/false)
   */
  public boolean getLimitSwtich(DigitalInput limitSwitch){
    return limitSwitch.get();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

} 
