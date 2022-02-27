// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.Limelight;
import frc.robot.subsystems.Drivetrain;
public class AlignToTargetDT extends CommandBase {
  Drivetrain drivetrain;
  Limelight limelight;
  double kp;
  /** Creates a new AlignToTargetDT. */
  public AlignToTargetDT(Drivetrain dt, Limelight ll) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = dt;
    limelight = ll;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drivetrain.cheesyIshDrive(0, wheel, quickTurn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
