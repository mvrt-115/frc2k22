// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Shooter.ShooterState;

public class MovingShot extends CommandBase {

  private Shooter shooter;
  private Storage storage;
  private Drivetrain drivetrain;
  private Turret turret;

  /** Creates a new MovingShot. */
  public MovingShot(Drivetrain drivetrain, Turret turret, Storage storage, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.turret = turret;
    this.shooter = shooter;
    this.storage = storage;

    addRequirements(drivetrain, turret, storage, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double driveSpeed = (drivetrain.getSpeeds().leftMetersPerSecond+drivetrain.getSpeeds().rightMetersPerSecond)/2;
    // double initSpeed = shooter.getVelocityFromWheelRPM();
    // double diff_const = initSpeed - driveSpeed*Math.cos(Math.toRadians(turret.getCurrentPositionDegrees()+limelight.getHorizontalOffset()));
    // double vert_drive_comp = driveSpeed*Math.sin(Math.toRadians(turret.getCurrentPositionDegrees()+limelight.getHorizontalOffset()));
    // double addSpeed = Math.sqrt(vert_drive_comp*vert_drive_comp + diff_const*diff_const)-initSpeed;
    
    // double totRPM = shooter.getRequiredRPM()+shooter.getRPMFromVelocity(addSpeed);
    
    
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
