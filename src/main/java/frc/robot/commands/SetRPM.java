// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
  private boolean given;
  private JoystickButton button;
  private Turret turret;

  public SetRPM(Shooter shooter, Storage storage, Turret turret) {
    this.shooter = shooter;
    this.storage = storage;
    this.turret = turret;
    addRequirements(shooter, storage);
  }


  public SetRPM(Shooter shooter, Storage storage, JoystickButton button) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.storage = storage;
    given = false;
    this.button = button;
    addRequirements(shooter, storage);
    
  }

  public SetRPM(Shooter shooter, Storage storage, double rpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.storage = storage;
    this.rpm = rpm;
    given = true;
    addRequirements(shooter, storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!given)
      rpm = shooter.getRequiredRPM();
    // rpm = SmartDashboard.getNumber("new rpm", defaultRPM);
    storage.setReadyShoot(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(given)
      rpm = shooter.getCurrentRPM();
    // shooter.setTargetRPM(rpm);
    // if(!storage.getBallColor().trim().equals(alliance)){
    //   shooter.setTargetRPM(600);
    // }
    // else {
    shooter.setTargetRPM(rpm);
    // }
    shooter.runMotor(0.5);
    // SmartDashboard.putNumber("new rpm", rpm);
    // SmartDashboard.putBoolean("changing rpm", true);

    if(rpm == 0)
       storage.runMotor(0);
    if(shooter.getState() == ShooterState.ATSPEED && turret.canShoot()) {
      storage.runMotor(1);
    } else
      storage.runMotor(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("changing rpm", false);
    shooter.setState(ShooterState.OFF);
    storage.setReadyShoot(false);
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
