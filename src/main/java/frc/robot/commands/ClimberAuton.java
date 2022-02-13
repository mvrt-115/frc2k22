// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimberAuton extends CommandBase {
  public Climber climber;
  public TalonFX motor;
  public double position;
  public DigitalInput[] sensor; 
  /** Creates a new ClimberAuton. */
  public ClimberAuton(Climber climber, TalonFX motor, double position, DigitalInput... sensor) {
    // Use addRequirements() here to declare subsystem dependencies.
    // initializing all of the field variables
    this.motor = motor;
    this.position = position;
    this.sensor = sensor;
    this.climber = climber;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //runs the motor using PID to given position, and calculates respective FeedForward constant
    double feedForward = 0; 
    if(motor.equals(climber.leftTelescopic)) feedForward = Constants.Climber.kFeedForwardTele;
    else feedForward = Constants.Climber.kFeedForwardPivot * Math.sin(climber.getPivotAngle() * 180 / Math.PI);

    climber.setPosition(motor, position, feedForward);
  }

  // Called once the command ends or is interrupted.
  // stops the motor when the command is finished
  @Override
  public void end(boolean interrupted) {
    //stops motor
    climber.stopMotor(motor);
  }

  // Returns true when the command should end.
  /** Checks to see if the the sensor on the moo
   * 
   */
  @Override
  public boolean isFinished() {
    if(motor.getSelectedSensorPosition() >= position - Constants.Climber.Auton.kAcceptablePIDError) { // if within acceptable error from constant position
      if(sensor.length >= 1) { //if sensor is needed to be detected
        if(climber.detectAllSensors(sensor)) //if the necessary sensor is activated, end command
          return true;
        else {
          new ClimberCompensatePIDError(climber, motor, sensor); //else, compensate for error by calling other method and continue command
          return false;
        } 
      }
      return true; //if acceptable error has been reached and no sensor is necessary, then end command
    }
    return false; //else continue command
  }
}
