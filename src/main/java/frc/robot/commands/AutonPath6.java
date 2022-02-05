// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutonPath6 extends CommandBase {
  /** Creates a new AutonPath6. */

  private Drivetrain drivetrain;
  public AutonPath6(Drivetrain dr) 
  {
    drivetrain = dr;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setDrivetrainMotorSpeed(0.3, 0.3);
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
    return false;
  }
}
