// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Runs the pivot arm motor using positional PID until final angle is reached, and
 *  if rung hasn't been touched, motors are run at constant speed to activate limit switches
 * @param climber      climber subsystem object to access subsystem methods
 * @param angle     position which telescopic arm needs to reach
 */
public class PivotArmAutonSensor extends CommandBase {
  public Climber climber;
  public double angle;

  /** Creates a new PivotArmAutonSensor. */
  public PivotArmAutonSensor(Climber climber, double angleIn) {
    climber = new Climber();
    angle =  angleIn;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new PivotArmAuton(climber, angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.getLimitSwitch(climber.pivotLimit);
  }
}
