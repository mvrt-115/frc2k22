// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class TelescopicRetractAuton extends CommandBase {
  /** Creates a new TelescopicFullRetract. */
  public Climber climber;
  public double positionFinal;

  public TelescopicRetractAuton(Climber climber2, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    climber = climber2;
    positionFinal = position;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setPosition(climber.leftTelescopic, positionFinal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopMotor(climber.leftTelescopic);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
