// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class PivotArmManual extends CommandBase {
  public Climber climber;
  public Supplier<Boolean> pivotButton;
  public double speed;

  public PivotArmManual(Climber climber2, Supplier<Boolean> pivotButton, double speedIn) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivotButton = pivotButton;
    climber = climber2;
    speed = speedIn;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setSpeed(climber.leftPivot, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopMotor(climber.leftPivot);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !pivotButton.get() || (speed < 0 && Constants.Climber.minPivotPos >= climber.getPivotAngle()) || 
      (speed > 0 && Constants.Climber.maxPivotPos <= climber.getPivotAngle());
  }
}
