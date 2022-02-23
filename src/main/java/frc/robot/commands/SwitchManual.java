// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;

public class SwitchManual extends CommandBase {
  /** Creates a new SwitchManual. */
  private Storage storage;

  public SwitchManual(Storage storage) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.storage = storage;
    addRequirements(storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    storage.setState(storage.currentState == Storage.StorageState.MANUAL ? Storage.StorageState.AUTOMATIC : Storage.StorageState.MANUAL);
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
    return true;
  }
}
