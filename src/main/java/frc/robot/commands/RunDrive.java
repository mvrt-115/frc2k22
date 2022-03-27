// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class RunDrive extends CommandBase {
  Drivetrain drivetrain;
  double start;
  double time;
  double out;
  int prevBalls;
  /** Creates a new RunDrive. */
  public RunDrive(Drivetrain dt, double time, double out){
    drivetrain = dt;
    this.time = time;
    this.out = out;
    addRequirements(drivetrain);
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
    drivetrain.setDrivetrainMotorSpeed(out, out);
    SmartDashboard.putBoolean("running", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopDrivetrain();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(start - Timer.getFPGATimestamp()) > time);
  }
}
