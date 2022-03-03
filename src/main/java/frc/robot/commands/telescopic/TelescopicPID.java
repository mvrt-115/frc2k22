// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.telescopic;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class TelescopicPID extends CommandBase {

  public Climber climber;
  public DigitalInput[] sensor;
  /** Creates a new ClimberCompensatePIDError. */
  public TelescopicPID(Climber climber, DigitalInput[] sensor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.sensor = sensor;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  // sets the speed of the telescopic arm to a slower on as the arm reaches the rung
  @Override
  public void initialize() {
    climber.setTelescopicSpeed(Constants.Climber.kApproachRungSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  // stops the telescopic motors
  @Override
  public void end(boolean interrupted) {
    climber.stopTelescopicMotor();
  }

  // Returns true when the command should end.
  // if all the sensors on the arms have been detected then the command is finished
  @Override
  public boolean isFinished() {
    return climber.detectAllSensors(sensor);
  }
}
