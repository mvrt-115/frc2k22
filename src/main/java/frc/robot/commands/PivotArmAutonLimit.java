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
public class PivotArmAutonLimit extends CommandBase {

  public Climber climber;
  public double angle;

  /** Creates a new PivotArmAutonSensor. */
  public PivotArmAutonLimit(Climber climber, double angleIn) {

    // initializes the field variables to the given parameters
    climber = new Climber();
    angle =  angleIn;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  // runs the pivot arm auton command to the angle given 
  @Override
  public void initialize() {
    new PivotArmAuton(climber, angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  // stops the pivting motor once the command is finished 
  @Override
  public void end(boolean interrupted) {
    climber.stopMotor(climber.pivot);
  }

  // Returns true when the command should end.
  // if the pivoting limit swtich is contacted before the pivot angle is reached, then the pivot stops
  @Override
  public boolean isFinished() {
    return climber.getLimitSwitch(climber.pivotLimit);
  }
}
