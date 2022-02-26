// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class SetHoodAngle extends CommandBase {
  private Shooter shooter;
  private double angle;

  /** Creates a new SetHoodAngle. */
  public SetHoodAngle(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angle = SmartDashboard.getNumber("Set Hood Angle", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // shooter.setTargetAngle(angle);
    angle = SmartDashboard.getNumber("Set Hood Angle", angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("Set Hood Angle", angle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
