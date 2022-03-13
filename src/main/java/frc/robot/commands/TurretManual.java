// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Turret.TurretState;

public class TurretManual extends CommandBase {
  private Turret turret;

  private double angle;

  private TurretState prevState;

  /** 
   * Creates a new TurnTurret
   * @param turret The turret subsystem
   * @param angle The angle to turn; should be bounded by -180 to 180 degrees
   */
  public TurretManual(Turret turret, double angle) {
    this.turret = turret;

    this.angle = angle;

    prevState = turret.getTurretState();

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(angle > Constants.Turret.kMaxAngle)
      angle = Constants.Turret.kMaxAngle;
    else if(angle < Constants.Turret.kMinAngle)
      angle = Constants.Turret.kMinAngle;

    SmartDashboard.putNumber("Turret Manual Degrees", angle);

    turret.setState(TurretState.DISABLED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.turnDegrees(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setPercentOutput(0);

    turret.setState(prevState);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(turret.getCurrentPositionDegrees() - angle) <= 2;
  }
}
