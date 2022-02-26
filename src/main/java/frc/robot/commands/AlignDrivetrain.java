// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Limelight;

public class AlignDrivetrain extends CommandBase {
  private Drivetrain drivetrain;
  private Limelight limelight;

  /** Creates a new AlignDrivetrain. */
  public AlignDrivetrain(Drivetrain drivetrain, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.drivetrain = drivetrain;
    this.limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double kP = 0.1;

    double e = limelight.getHorizontalOffset();
    
    double output = kP * e;

    drivetrain.setOutputVoltage(output, -output);
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
