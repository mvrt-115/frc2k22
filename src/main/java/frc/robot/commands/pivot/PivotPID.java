// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class PivotPID extends CommandBase {
  public Climber climber;
  public DigitalInput[] sensor;
  /** Creates a new ClimberCompensatePIDError. */
  public PivotPID(Climber climber,DigitalInput[] sensor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.sensor = sensor;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setPivotSpeed(Constants.Climber.kApproachRungSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopPivotMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.detectAllSensors(sensor);
  }
}
