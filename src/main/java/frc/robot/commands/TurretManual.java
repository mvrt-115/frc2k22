// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Turret.TurretState;

public class TurretManual extends CommandBase {
  private Turret turret;

  private double percentOut;
  private Supplier<Boolean> isFinished;

  /** Creates a new TurnTurret. */
  public TurretManual(Turret turret, double percentOut, Supplier<Boolean> isFinished) {
    this.turret = turret;

    this.percentOut = percentOut;
    this.isFinished = isFinished;

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.setState(TurretState.DISABLED);

    SmartDashboard.putNumber("manual", percentOut);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.setPercentOutput(percentOut);

    SmartDashboard.putNumber("manual", percentOut);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("interrupted", percentOut);
    turret.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished.get();
  }
}