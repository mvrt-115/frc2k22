// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;

public class ShootFromStorage extends CommandBase {
  /** Creates a new ShootFromStorage. */
  private Storage storage;
  public ShootFromStorage(Storage storageIn) {
    storage = storageIn;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    storage.setOverriden(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    storage.getMotor().set(ControlMode.PercentOutput, 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    storage.getMotor().set(ControlMode.PercentOutput, 0);
    storage.setOverriden(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return storage.getBalls() == 0;
  }
}
