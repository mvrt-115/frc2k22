// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.element.ModuleElement.Directive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Limelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;

public class TargetDrive extends CommandBase {

  Drivetrain drivetrain;
  Turret turret;
  double dir;

  /** Creates a new TargetDrive. */
  public TargetDrive(Drivetrain drivetrain, Turret turret)
  {
    this.drivetrain = drivetrain;
    this.turret = turret;
    dir = 0;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(turret.getCurrentPositionDegrees()>0)
      dir = 1;
    else
      dir = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    drivetrain.setDrivetrainMotorSpeed(dir*Constants.Drivetrain.AUTO_TURN_SPEED, -1*dir*Constants.Drivetrain.AUTO_TURN_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turret.getLimelight().targetsFound();
  }
}
