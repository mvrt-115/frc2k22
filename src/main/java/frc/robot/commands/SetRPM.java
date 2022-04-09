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
  private boolean given;
  private JoystickButton button;
  private Turret turret;
  private boolean dash;
  public boolean auton = false;
  boolean shoot = false;

  boolean low = false;

  public SetRPM(Shooter shooter, Storage storage, Turret turret) {
    this.shooter = shooter;
    this.storage = storage;
    this.turret = turret;
    addRequirements(shooter, storage);
  }


  public SetRPM(Shooter shooter, Storage storage, Turret turret, JoystickButton button) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.storage = storage;
    this.turret = turret;
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

  
  public SetRPM(Shooter shooter, Storage storage, boolean storEnd, double hi) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.storage = storage;
    auton = true;
    // SmartDashboard.putNumber("new rpm", 0);
    addRequirements(shooter, storage);
  }
  public SetRPM(Shooter shooter, Storage storage, boolean storEnd, boolean hi) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.storage = storage;
    
    // SmartDashboard.putNumber("new rpm", 0);
    addRequirements(shooter, storage);
  }
    public SetRPM(Shooter shooter, Storage storage, boolean dash) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.storage = storage;
    this.dash = dash;
    // SmartDashboard.putNumber("new rpm", 0);
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
      // rpm = shooter.getRequiredRPM();
      if(dash) {
      // rpm = SmartDashboard.getNumber("new rpm", 0);

      if(shooter.getState() == ShooterState.ATSPEED)
        shoot = true;
      else 
        shoot = false;
      if(shoot)
        storage.runMotor(1);
      else
        storage.runMotor(0);
    }
    if(given){
      shooter.setTargetRPM(rpm);
    }
    else{
      // shooter.setTargetRPM(1000);

      shooter.setTargetRPM(shooter.getRequiredRPM());//(shooter.getRequiredRPM());
    }

    if(rpm == 0)
       storage.runMotor(0);
    if(shooter.getState() == ShooterState.ATSPEED) {
        storage.runMotor(1);
    } else
      storage.runMotor(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // SmartDashchanging rpm", false);
    shooter.setState(ShooterState.OFF);
    storage.setReadyShoot(false);
    storage.runMotor(0);
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
