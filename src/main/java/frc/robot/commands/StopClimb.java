// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class StopClimb extends CommandBase {
  /** Creates a new StopClimb. */
  public Climber climber;
  public Supplier<Boolean> getStopState;
  public StopClimb(Climber climber, Supplier<Boolean> getStopState) {
    this.climber = climber;
    this.getStopState = getStopState;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stops all motors
    climber.stopMotor(climber.leftTelescopic);
    // climber.stopMotor(climber.pivot);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getStopState.get(); //checks if stop button is pressed
  }
}
