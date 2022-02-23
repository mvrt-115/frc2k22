// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class SetRPM extends CommandBase {
  /** Creates a new SetRPM. */
  private Shooter shooter;
  private double rpm;

  public SetRPM(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double defaultRPM = shooter.getRequiredRPM();
    rpm = SmartDashboard.getNumber("new rpm", 100);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setTargetRPM(-rpm);
    SmartDashboard.putNumber("new rpm", rpm);
    SmartDashboard.putBoolean("changing rpm", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("changing rpm", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

/**
 * stuff:
 *  // double tempRPM = SmartDashboard.getNumber("target rpm", targetRPM);
    // targetRPM = tempRPM;
    // setState(ShooterState.SPEEDING);
    // flywheelLeader.set(ControlMode.Velocity, rpmToTicks(targetRPM));
 */