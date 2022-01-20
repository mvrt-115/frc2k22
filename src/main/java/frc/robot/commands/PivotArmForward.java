// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class PivotArmForward extends CommandBase {
  /** Creates a new PivotArmForward. */
  public Climber climber;

  public PivotArmForward(Climber climberIn) {
    // Use addRequirements() here to declare subsystem dependencies.
    climber = climberIn;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setSpeed(climber.pivot, 0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopMotor(climber.pivot);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotContainer.getForwardArm();
  }
}
