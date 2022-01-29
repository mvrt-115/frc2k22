// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class TelescopicArmAutonSensor extends CommandBase {
  /** Creates a new TelescopicArmAutonSensor. */
  public Climber climber;
  public double position;

  public TelescopicArmAutonSensor(Climber climber, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.position = position;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new TelescopicArmAuton(climber, position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setSpeed(climber.leftTelescopic, Constants.Climber.approachRungSpeed);
   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopMotor(climber.leftTelescopic);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.getLimitSwitch(climber.leftTelescopicLimit) && climber.getLimitSwitch(climber.rightTelescopicLimit);
  }
}
