// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Intake.IntakeState;

public class StopIntaking extends CommandBase {
  /** Creates a new StopIntaking. */
  private Intake intake;
  private Storage storage;
  public StopIntaking(Intake intakeIn, Storage storageIn) {
    intake = intakeIn;
    storage = storageIn;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setState(IntakeState.PIVOTING_UP);
    storage.setIntaking(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // don't modify this command!
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}