// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;

public class TrackBalls extends CommandBase {

  Storage storage;
  Shooter shooter;
  String alliance = DriverStation.getAlliance().toString();
  boolean stopExp = false;
  /** Creates a new TrackBalls. */
  public TrackBalls(Storage st, Shooter shoot) {
    storage = st;
    shooter = shoot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(storage, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!storage.getShooting()){
        if(!storage.getBallColor().trim().equals("No Ball") && !storage.getBallColor().trim().equals(alliance)){
          SmartDashboard.putBoolean("pooping", true);
          shooter.setTargetRPM(100);
          storage.runMotor(1);
          stopExp = true;
        } else if (stopExp) {
          SmartDashboard.putBoolean("pooping", false);
          shooter.setTargetRPM(0);
          storage.runMotor(0);
          stopExp = false; 
        }
    }
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
