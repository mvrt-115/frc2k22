// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter.ShooterState;

public class SetRPMLaunchpad extends CommandBase {

  private Shooter shooter;
  private Storage storage;
  private JoystickButton button;
  private double start = 0;
  private double rpm;
  private Turret turret;


  /** Creates a new SetRPMDash. */
  public SetRPMLaunchpad(Shooter shooter, Storage storage, Turret turret, JoystickButton jb) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.storage = storage;
    this.turret = turret;
    button = jb;
    rpm = 0;
    // SmartDashboard.putNumber("new rpm", 0);
    addRequirements(shooter, storage);
  }
  public SetRPMLaunchpad(Shooter shooter, Storage storage) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.storage = storage;
    this.turret = turret;
    // button = jb;
    rpm = 0;
    // SmartDashboard.putNumber("new rpm", 0);
    addRequirements(shooter, storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    // System.out.println("racism2");
    storage.setReadyShoot(true);
    rpm = shooter.getRequiredRPM();
    start = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    shooter.getMotor().set(ControlMode.PercentOutput, 0.7);
    if(Timer.getFPGATimestamp() - start > 1.0){

      storage.runMotor(0.4);
    }

    turret.setOffset(0);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    shooter.setState(ShooterState.OFF);
    storage.setReadyShoot(false);
    storage.runMotor(0);
    // System.out.println("hi5");

    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(button != null)
      return !button.get();
    return rpm == 0;
  }
}