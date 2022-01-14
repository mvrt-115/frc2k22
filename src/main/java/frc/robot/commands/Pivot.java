// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

public class Pivot extends CommandBase {
  /** Creates a new PivotIntake. */
  private Intake intake;

  public Pivot(Intake intakeIn) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = intakeIn;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intake.getState() == IntakeState.UP)
     intake.setState(IntakeState.PIVOTING_DOWN);
    else if(intake.getState() == IntakeState.INTAKING)
      intake.setState(IntakeState.PIVOTING_UP);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setState(intake.getState()==IntakeState.PIVOTING_DOWN ? IntakeState.INTAKING : IntakeState.UP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}