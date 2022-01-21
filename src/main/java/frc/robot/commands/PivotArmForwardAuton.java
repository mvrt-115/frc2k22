// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class PivotArmForward extends CommandBase {
  /** Creates a new PivotArmForward. */
  public Climber climber;
 // public Supplier<Boolean> forwardArm;
  public double position;

  public PivotArmForward(Climber climberIn, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    //this.forwardArm = forwardArm;
    this.position = position;
    this.climber = climberIn;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setPosition(climber.pivot, position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //so the conditions for the isFinished should be plates activated OR button released OR potentiometer reached max angle/distance
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
