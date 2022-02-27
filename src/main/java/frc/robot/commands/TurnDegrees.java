// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TurnDegrees extends CommandBase {
  private Drivetrain drivetrain;
  private double degrees;
  private double targetDegrees;
  private double lastTime;
  private double lastError;
  private double totalError;
  /** Creates a new TurnDegrees. */
  public TurnDegrees(Drivetrain drivetrain, double degrees) {
    this.drivetrain = drivetrain;
    this.degrees = degrees;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetDegrees = drivetrain.getGyroAngle().getDegrees() + degrees;
    lastTime = Timer.getFPGATimestamp();
    totalError = 0;
    lastError = degrees;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = targetDegrees - drivetrain.getGyroAngle().getDegrees();
    double dT = Timer.getFPGATimestamp() - lastTime;
    double dEdT = (error - lastError) / dT;
    
    totalError += error * dT;

    double output = 
      error * Constants.Drivetrain.kPTurn + 
      dEdT * Constants.Drivetrain.kDTurn + 
      totalError * Constants.Drivetrain.kITurn;

    drivetrain.setDrivetrainMotorSpeed(output, -output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setDrivetrainMotorSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(targetDegrees - drivetrain.getGyroAngle().getDegrees()) < 2;
  }
}
