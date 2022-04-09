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

public class SetRPMValue extends CommandBase {

  private Shooter shooter;
  private Storage storage;
  private double rpm;

  /** Creates a new SetRPMDash. */
  public SetRPMValue(Shooter shooter, Storage storage, double rpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.storage = storage;
    this.rpm = rpm;
    // SmartDashboard.putNumber("new rpm", 0);
    addRequirements(shooter, storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if(shooter.getState() == ShooterState.ATSPEED)
    {
      storage.runMotor(1);
      shooter.setTargetRPM(rpm);
    }
    else 
      storage.runMotor(0);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    shooter.setState(ShooterState.OFF);
    storage.setReadyShoot(false);
    storage.runMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
