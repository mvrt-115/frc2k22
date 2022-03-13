// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.telescopic;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class TelescopicRatchet extends CommandBase {
  
  public Climber climber;
  public double servoTurn;

  /** Creates a new TelescopicRatchet. */
  public TelescopicRatchet(Climber climber, double servoTurn) {
    this.climber = climber;
    this.servoTurn = servoTurn;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setTelescopicSpeed(Constants.Climber.kMotorInitialUnratchetSpeed);
    Timer.delay(Constants.Climber.kMotorDownTime);
  }

  // Called every time the scheduler runs while the command is scheduled.

  // ratchets both the servos
  @Override
  public void execute() {
    climber.setServoTurn(servoTurn);
    climber.stopTelescopicMotor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(climber.leftServo.getAngle() - servoTurn) <= Constants.Climber.kServoError;
  }
}
