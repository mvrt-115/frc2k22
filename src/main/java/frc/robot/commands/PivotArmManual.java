// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

/**
 * Runs the pivot arm motor at a sconstant speed until the manual button currently pressed is released, 
 *  or if the pivot has reached a maximum turn.
 * @param climber         climber subsystem object to access subsystem methods
 * @param pivotButton     supplier that gets state of the manual pivot button passed in
 * @param speed           speed at which the pivot arm should run at
 */
public class PivotArmManual extends CommandBase {

  public Climber climber; 
  public Supplier<Boolean> pivotButton; 
  public double speed;

  public PivotArmManual(Climber climber, Supplier<Boolean> pivotButton, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.

    // field variables are all initialized to given parameter values
    this.pivotButton = pivotButton;
    this.climber = climber;
    this.speed = speed;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  // sets the speed of the pivot arm's motor to the given speed
  @Override
  public void execute() {
    climber.setSpeed(climber.pivot, speed);
  }

  // Called once the command ends or is interrupted.
  // stops the motor when the button is released
  @Override
  public void end(boolean interrupted) {
    climber.stopMotor(climber.pivot);
  }

  // Returns true when the command should end.
  // if the button is released or the max or min angle that the pivot can do is reached
  @Override
  public boolean isFinished() {
    return !pivotButton.get() || (speed < 0 && Constants.Climber.Auton.kMinPivotPos >= climber.getPivotAngle()) || 
      (speed > 0 && Constants.Climber.Auton.kMaxPivotPos <= climber.getPivotAngle());
  }
}
