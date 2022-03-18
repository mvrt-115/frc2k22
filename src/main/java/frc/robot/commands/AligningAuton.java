// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Shooter;

public class AligningAuton extends SequentialCommandGroup {
  /** Creates a new AligningAuton. */
  public AligningAuton(Drivetrain dt, Storage str, Turret tur, Shooter shoot) {
    addCommands(
      new AlignIntakeToBall(dt, str).withTimeout(3),
      new SetRPM(shoot, str, tur)
    );
  }

 
}
