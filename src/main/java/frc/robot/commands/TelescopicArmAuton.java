// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Runs the telescopic arm motor using positional PID until final position is reached
 * @param climber      climber subsystem object to access subsystem methods
 * @param position     position which telescopic arm needs to reach
 */
public class TelescopicArmAuton extends CommandBase {
  /** Creates a new TelescopicFullExtend. */

  public Climber climber; // climber instance
  public double positionFinal; // position the telescopic arms should go to

  public TelescopicArmAuton(Climber climber, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    // initializes field variables to the given parameter values
    this.climber = climber;
    positionFinal = position;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  // sets the position of the telescopic arm to the given value
  @Override
  public void execute() {
    climber.setPosition(climber.leftTelescopic, positionFinal);
   }

  // Called once the command ends or is interrupted.
  // stops the telescopic motors once the button is released
  @Override
  public void end(boolean interrupted) {
    climber.stopMotor(climber.leftTelescopic);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
