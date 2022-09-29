// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Shooter.ShooterState;

public class SetRPMRequired extends CommandBase {

  private Shooter shooter;
  private Storage storage;
  private JoystickButton button;
  private double start = 0;
  private double rpm;
  private double firstAtSpeed = -1;
  boolean shoot = false;

  /** Creates a new SetRPMDash. */
  public SetRPMRequired(Shooter shooter, Storage storage, JoystickButton jb) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.storage = storage;
    button = jb;
    rpm = 0;
    // SmartDashboard.putNumber("new rpm", 0);
    addRequirements(shooter, storage);
  }

  public SetRPMRequired(Shooter shooter, Storage storage) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.storage = storage;
    // button = jb;
    rpm = 0;
    // SmartDashboard.putNumber("new rpm", 0);
    addRequirements(shooter, storage);
    shooter.autoOffset();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // storage.setReadyShoot(true); //WARN: setting true here makes storage run w/o getting up to speed
    rpm = shooter.getRequiredRPM();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rpm = shooter.getRequiredRPM();
    shooter.setTargetRPM(rpm);

    if(shooter.getState() == ShooterState.ATSPEED)
    {
      if(rpm != 0 && firstAtSpeed != -1) {
        // Storage.getInstance().setReadyShoot(true);
        firstAtSpeed = Timer.getFPGATimestamp();
      }

      if(Math.abs(firstAtSpeed - Timer.getFPGATimestamp()) > 0.5) {
        Storage.getInstance().setReadyShoot(true);
        shoot = true;
      }
        
      // storage.runMotor(0.4); 

      // turret.setState(TurretState.DISABLED);
    }
    else{ 
      // storage.runMotor(0);
      Storage.getInstance().setReadyShoot(false);
      firstAtSpeed = -1;
      shoot = false;
    }
    
    if(shoot)
      storage.runMotor(1);
    else
    {storage.runMotor(0);}

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setState(ShooterState.OFF);
    storage.setReadyShoot(false);
    storage.runMotor(0);
    // System.out.println("hi5");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (button != null)
      return !button.get();
    return rpm == 0;
  }
}