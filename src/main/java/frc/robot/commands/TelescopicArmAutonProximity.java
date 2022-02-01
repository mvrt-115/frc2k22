// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class TelescopicArmAutonProximity extends CommandBase {
  public Climber climber;
  public double position;
  /** Creates a new TelescopicArmAutonProximity. */
  public TelescopicArmAutonProximity(Climber climberIn, double positionIn) {
    // Use addRequirements() here to declare subsystem dependencies.
    climber = climberIn;
    position = positionIn;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new TelescopicArmAuton(climber, position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.getProximity(climber.leftTelescopicProximity) && climber.getProximity(climber.rightTelescopicProximity) && climber.getProximity(climber.pivotProximity);
  }
}
