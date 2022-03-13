// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.telescopic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class TelescopicUnratchet extends CommandBase {
  public Climber climber;
  /** Creates a new TelescopicUnratchet. */
  public TelescopicUnratchet(Climber climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  // unratchets both the servos
  @Override
  public void execute() {
    climber.rightServo.set(Constants.Climber.kServoUnRatchet);
    climber.leftServo.set(Constants.Climber.kServoUnRatchet);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}