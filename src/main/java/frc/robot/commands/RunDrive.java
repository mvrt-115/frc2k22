// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Turret;

public class RunDrive extends CommandBase {
  Drivetrain drivetrain;
  Intake intake;
  Storage storage;
  Timer time;
  Shooter shooter;
  Turret turret;
  double start;
  /** Creates a new RunDrive. */
  public RunDrive(Drivetrain dt, Intake in, Storage stor, Shooter shooter, Turret turret){
    drivetrain = dt;
    intake = in;
    storage = stor;
    this.shooter = shooter;
    this.turret = turret;
    new Pivot(in, stor).schedule();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("running", false);
    start = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drivetrain.cheesyIshDrive(.3, 0, false);
    drivetrain.setDrivetrainMotorSpeed(0.2, 0.2);
    SmartDashboard.putBoolean("running", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("running", false);
    new PivotUp(intake, storage).schedule();
    drivetrain.stopDrivetrain();
    new SetRPM(shooter, storage, 3000).schedule();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(start - Timer.getFPGATimestamp()) > 4;
    // return false;
  }
}
