// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

/**
 * Runs the telescopic arm motor using positional PID until final position is reached, and
 *  if rung hasn't been touched, motors are run at constant speed to activate limit switches
 * @param climber      climber subsystem object to access subsystem methods
 * @param position     position which telescopic arm needs to reach
 */
public class TelescopicArmAutonLimit extends CommandBase {
  /** Creates a new TelescopicArmAutonSensor. */
  public Climber climber; // climber instance
  public double position; // the extent the telescopic arm wants to be extended/retracted

  public TelescopicArmAutonLimit(Climber climber, double position) {
    // Use addRequirements() here to declare subsystem dependencies.

    // initializes the field variables to the given parameter values
    this.climber = climber;
    this.position = position;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  // runs the original pivot auton to the given angle
  @Override
  public void initialize() {
    new PivotArmAuton(climber, position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  // when the position that the climber wants to be at is reached, the pivot motors are stopped
  @Override
  public void end(boolean interrupted) {
    climber.stopMotor(climber.pivot);
  }

  // Returns true when the command should end.
  // finishes the telescopic auton command if the limit swtiches detect the rung before angle given is reached 
  @Override
  public boolean isFinished() {
    return climber.getLimitSwitch(climber.leftTelescopicLimit) && climber.getLimitSwitch(climber.rightTelescopicLimit);
  }
}
