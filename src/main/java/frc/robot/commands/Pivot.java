// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Intake.IntakeState;

public class Pivot extends CommandBase {
  /** Creates a new Pivot. */
  private Intake intake;
  private Storage storage;
  public Pivot(Intake intake, Storage storage) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.storage=  storage;
    addRequirements(intake, storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(intake.getPivotState()){
      intake.setState(IntakeState.PIVOTING_DOWN);
      storage.setIntaking(true);
    }
    else{
      intake.setState(IntakeState.PIVOTING_UP);
      storage.setIntaking(false);
    }
    intake.setPivotState(!intake.getPivotState());
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
