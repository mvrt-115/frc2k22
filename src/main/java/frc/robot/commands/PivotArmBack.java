// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class PivotArmBack extends CommandBase {
  /** Creates a new PivotArmBack. */
  public Climber climber;
  public Supplier<Boolean> armBack;
  
  public PivotArmBack(Climber climberIn, Supplier<Boolean> armBack) {
    // Use addRequirements() here to declare subsystem dependencies.
    climber = climberIn;
    this.armBack = armBack;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    climber.setSpeed(climber.pivot, -0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopMotor(climber.pivot);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !armBack.get();
  }
}
