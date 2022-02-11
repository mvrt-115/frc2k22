// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimberCompensatePIDError extends CommandBase {
  public Climber climber;
  public TalonFX motor;
  public DigitalInput[] sensor;
  /** Creates a new ClimberCompensatePIDError. */
  public ClimberCompensatePIDError(Climber climber, TalonFX motor, DigitalInput[] sensor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.sensor = sensor;
    this.motor = motor;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setSpeed(motor, Constants.Climber.kApproachRungSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopMotor(motor);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.detectAllSensors(sensor);
  }
}
