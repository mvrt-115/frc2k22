// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;

public class ManualStorage extends CommandBase {
  Storage storage;
  /** true for up, false for down. */
  boolean direction;
  Supplier<Boolean> button;
  /** Creates a new ManualStorage. */
  public ManualStorage(Storage str, boolean direction, Supplier<Boolean> button ) {
    // Use addRequirements() here to declare subsystem dependencies.
    storage = str;
    this.direction = direction;
    this.button = button;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    storage.setOverriden(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(direction){
      storage.runMotor(0.7);
    }
    else {
      storage.runMotor(-0.7);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    storage.setOverriden(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !button.get();
  }
}
