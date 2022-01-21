// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TelescopicRetractManual extends CommandBase {
  public Climber climer;
  public Supplier<Boolean> teleRetract;
  /** Creates a new TelescopicExtendManual. */
  public TelescopicRetractManual(Climber climberIn, Supplier<Boolean> teleRetract) {
    this.climber = climberIn;
    this.teleRetract = teleRetract;
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setSpeed(-0.1);
  }

  // Called once the command ends or is interrupted.
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
