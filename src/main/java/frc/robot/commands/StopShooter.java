// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Shooter.ShooterState;

public class StopShooter extends CommandBase {
  Shooter shooter;
  Storage storage;

  /** Creates a new StopShooter. */
  public StopShooter(Shooter shooter, Storage storage) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, storage);
    this.shooter = shooter;
    this.storage = storage;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.stopFlywheel();
    shooter.setState(ShooterState.OFF);
    storage.runMotor(0);System.out.println("hi8");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
