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
import frc.robot.subsystems.Shooter.ShooterState;

public class SetRPM extends CommandBase {
  /** Creates a new SetRPM. */
  private Shooter shooter;
  private double rpm;
  private Storage storage;
  private JoystickButton button;
  private Turret turret;
  boolean shoot = false;

  public SetRPM(Shooter shooter, Storage storage, Turret turret) {
    this.shooter = shooter;
    this.storage = storage;
    this.turret = turret;
    addRequirements(shooter, storage);
  }


  public SetRPM(Shooter shooter, Storage storage, Turret turret, JoystickButton button) {
    this.shooter = shooter;
    this.storage = storage;
    this.turret = turret;
    this.button = button;
    addRequirements(shooter, storage);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rpm = shooter.getRequiredRPM();
    // rpm = SmartDashboard.getNumber("new rpm", defaultRPM);
    storage.setReadyShoot(true);
    System.out.println("racism");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.getState() == ShooterState.ATSPEED)
      shoot = true;
    else 
      shoot = false;
    
    if(shoot)
      storage.runMotor(1);
    else
    {storage.runMotor(0);
    System.out.println("hi1");}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // SmartDashchanging rpm", false);
    shooter.setState(ShooterState.OFF);
    storage.setReadyShoot(false);
    storage.runMotor(0);
    System.out.println("hi2");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(button != null)
      return !button.get();
    return rpm == 0;
  }
}

/**
 * stuff:
 *  // double tempRPM = SmartDashboard.getNumber("target rpm", targetRPM);
    // targetRPM = tempRPM;
    // setState(ShooterState.SPEEDING);
    // flywheelLeader.set(ControlMode.Velocity, rpmToTicks(targetRPM));
 */
