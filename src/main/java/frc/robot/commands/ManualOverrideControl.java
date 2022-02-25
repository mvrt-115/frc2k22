// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;

public class ManualOverrideControl extends CommandBase {
  /** Creates a new ManualOverrideControl. */
  private Supplier<Double> throttle;
  private Storage storage;
  public ManualOverrideControl(Storage storage, Supplier<Double> throttle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.storage = storage;
    addRequirements(storage);
    this.throttle = throttle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(storage.currentState == Storage.StorageState.MANUAL){
      if(Math.abs(throttle.get())>0.03){
        storage.runMotor(throttle.get());
      }
    }
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
