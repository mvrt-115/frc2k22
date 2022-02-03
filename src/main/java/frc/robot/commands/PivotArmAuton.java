// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Runs the pivot arm motor using positional PID on the angle turn until final angle is reached
 * @param climber      climber subsystem object to access subsystem methods
 * @param angle     position which telescopic arm needs to reach
 */
public class PivotArmAuton extends CommandBase {
  /** Creates a new PivotArmBack. */
  public Climber climber;
  public double finalAngle;
  
  public PivotArmAuton(Climber climber, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.

    // initializing all of the field variables to the parameter values
    this.climber = climber;
    finalAngle = angle;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  // sets the position of the pivoting arm to the given angle
  @Override
  public void execute() {
    climber.setPosition(climber.pivot, finalAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // so the conditions for the isFinished should be plates activated OR button released OR potentiometer reached max angle/distance
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
