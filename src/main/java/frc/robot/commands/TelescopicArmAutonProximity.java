// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class TelescopicArmAutonProximity extends CommandBase {

  public Climber climber; // climber instance
  public double position; // position telescopic arms should go to

  /** Creates a new TelescopicArmAutonProximity. */
  public TelescopicArmAutonProximity(Climber climberIn, double positionIn) {
    // Use addRequirements() here to declare subsystem dependencies.

    // field variables are initialized to the given parameter values
    climber = climberIn; 
    position = positionIn;
  }

  // Called when the command is initially scheduled.
  // the telescopic arm command is ran to the given position value
  @Override
  public void initialize() {
    new TelescopicArmAuton(climber, position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  // sets the speed of the telescopic arms' motors to the given speed
  @Override
  public void execute() {
    climber.setSpeed(climber.leftTelescopic, Constants.Climber.kApproachRungSpeed);
  }

  // Called once the command ends or is interrupted.
  // when the position that the climber wants to be at is reached, the motors are stopped
  @Override
  public void end(boolean interrupted) {
    climber.stopMotor(climber.leftTelescopic);
  }

  // Returns true when the command should end.
  // finishes the telescopic auton command if the proximity sensor senses the rung before the arm gets to the given position
  @Override
  public boolean isFinished() {
    return climber.getProximity(climber.leftTelescopicProximity) && climber.getProximity(climber.rightTelescopicProximity) && climber.getProximity(climber.pivotProximity);
  }
}
