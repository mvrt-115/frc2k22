// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.telescopic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class LowClimbManual extends CommandBase {
  Climber climber;
  /** Creates a new LowClimb. */
  public LowClimbManual(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setClimb(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setTelescopicSpeed(Constants.Climber.kTelescopicExtendManualSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopAllMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.getTelescopicPosition() >= 14300;
  }
}
