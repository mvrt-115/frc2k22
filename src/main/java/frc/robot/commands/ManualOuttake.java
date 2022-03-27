// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Storage;

public class ManualOuttake extends CommandBase {
  Storage storage;
  Shooter shooter;
  Supplier<Boolean> button;
  /** Creates a new ManualOuttake. */
  public ManualOuttake(Storage storage, Shooter shooter, Supplier<Boolean> button) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.storage = storage;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setTargetRPM(700);
    storage.setOverriden(true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.getState() == ShooterState.ATSPEED){
      storage.runMotor(0.8);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setTargetRPM(0);
    storage.setOverriden(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return button.get();
  }
}
