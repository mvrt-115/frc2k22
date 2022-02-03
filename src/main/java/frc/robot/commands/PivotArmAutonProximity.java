// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class PivotArmAutonProximity extends CommandBase {

  public Climber climber; // climber instance
  public double position; // position of the telescopic arm

  /** Creates a new PivotArmAutonProximity. */
  public PivotArmAutonProximity(Climber climberIn, double positionIn) {
    // Use addRequirements() here to declare subsystem dependencies.
    // initializes the field variables to the given parameters
    climber = climberIn;
    position = positionIn;
  }

  // Called when the command is initially scheduled.
  // runs the telescopic arm auton command to the given angle
  @Override
  public void initialize() {
    new TelescopicArmAuton(climber, position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  // stops the telescopic motors once the coommand is finished 
  @Override
  public void end(boolean interrupted) {
    climber.stopMotor(climber.leftTelescopic);
  }

  // Returns true when the command should end.
  /** stops the telelscopic motors if the pivot proximity sensor
   *  senses the rung before the arm gets to the given angle */ 
  @Override
  public boolean isFinished() {
    return climber.getProximity(climber.pivotProximity) && climber.getProximity(climber.leftTelescopicProximity) && climber.getProximity(climber.rightTelescopicProximity);
  }
}
