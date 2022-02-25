// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AlignIntakeToBall extends CommandBase {
  /** Creates a new AlignIntakeToBall. */

  private boolean notStopping;
  private Drivetrain drivetrain;

  public AlignIntakeToBall(Drivetrain drivetrain2, boolean _notStopping) 
  {
    drivetrain = drivetrain2;
    notStopping = _notStopping;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //drivetrain.alignToBall();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setDrivetrainMotorSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !notStopping;
  }
}
