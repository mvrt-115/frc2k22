// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Turret.TurretState;

public class EstimateTurret extends CommandBase {
  private Turret turret;
  private Drivetrain drivetrain;
  
  /** Creates a new EstimateTurret. */
  public EstimateTurret(Turret turret, Drivetrain drivetrain) {
    this.turret = turret;
    this.drivetrain = drivetrain;    

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.setState(TurretState.DISABLED);
    drivetrain.setOdometry(new Pose2d(5.86, 5.1, Rotation2d.fromDegrees( drivetrain.getGyroAngle().getDegrees()))); // debuggubg
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.estimate(drivetrain.getPose());
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
