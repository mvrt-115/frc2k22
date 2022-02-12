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
    this.motor = motor;
    this.position = position;
    this.sensor = sensor;
    this.climber = climber;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double feedForward = 0;
    if(motor.equals(climber.leftTelescopic)) feedForward = Constants.Climber.kFeedForwardTele;
    else feedForward = Constants.Climber.kFeedForwardPivot;

    climber.setPosition(motor, position, feedForward);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setSpeed(motor, Constants.Climber.kApproachRungSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopMotor(motor);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(motor.getSelectedSensorPosition() >= position - 0.05) {
      if(sensor.length >= 1) {
        if(climber.detectAllSensors(sensor))
          return true;
        else {
          new ClimberCompensatePIDError(climber, motor, sensor);
          return false;
        } 
      }
      return true;
    }
    return false;
  }
}