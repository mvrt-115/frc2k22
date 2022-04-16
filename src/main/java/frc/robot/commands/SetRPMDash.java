// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Turret;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Turret.TurretState;

public class SetRPMDash extends CommandBase {

  private Shooter shooter;
  private Storage storage;
  private Drivetrain drivetrain;
  private double rpm;
  private double offset;
  private Turret turret;

  /** Creates a new SetRPMDash. */
  public SetRPMDash(Shooter shooter, Storage storage, Turret turret, Drivetrain drivetain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.storage = storage;
    this.turret = turret;
    this.drivetrain = drivetrain;
    rpm = 0;
    SmartDashboard.putNumber("new rpm", 0);
    addRequirements(shooter, storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    rpm = SmartDashboard.getNumber("new rpm", 0);

    // storage.setReadyShoot(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    rpm = SmartDashboard.getNumber("new rpm", 0);
    shooter.setTargetRPM(rpm);

    if(turret.canShoot())
    {
      turret.setState(TurretState.DISABLED);
    }
    else
    {
      turret.setState(TurretState.TARGETING);
    }

    if(shooter.getState() == ShooterState.ATSPEED)
    {
    System.out.println("racism1");
    storage.setReadyShoot(true);
      storage.runMotor(0.4); 

      turret.setState(TurretState.DISABLED);
    }
    else 
      // storage.runMotor(0);
      storage.setReadyShoot(false);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    shooter.setState(ShooterState.OFF);
    storage.setReadyShoot(false);
    storage.runMotor(0);
    System.out.println("hi3");
    turret.setState(TurretState.TARGETING);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
