// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;

public class TrackBalls extends CommandBase {

  Storage storage;
  Shooter shooter;
  String alliance;
  /** Creates a new TrackBalls. */
  public TrackBalls(Storage st, Shooter shoot, String alli) {
    storage = st;
    shooter = shoot;
    alliance = alli;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!storage.getShooting()){
      if(!storage.getBallColor().equals("No Ball")){
        if(!storage.getBallColor().equals(alliance)){
          new SetRPM(shooter, storage, 200);
        }
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
