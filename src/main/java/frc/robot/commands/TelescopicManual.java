// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

/**
 * Runs the telescopic arm motor a constant speed until the manual button currently pressed is released, 
 *  or if the telescopic has reached a maximum turn.
 * @param climber        climber subsystem object to access subsystem methods
 * @param teleButton     supplier that gets state of the manual telescopic button passed in
 * @param speed          speed at which the pivot arm should run at
 */
public class TelescopicManual extends CommandBase {
  public Climber climber;
  public Supplier<Boolean> teleButton;
  public double speed;

  /** Creates a new TelescopicExtendManual. */
  public TelescopicManual(Climber climber, Supplier<Boolean> teleButton, double speed) {
    this.climber = climber;
    this.teleButton = teleButton;
    this.speed = speed;
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setSpeed(climber.leftTelescopic, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopMotor(climber.leftTelescopic);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !teleButton.get() || (speed < 0 && Constants.Climber.kTelescopicFullRetract >= climber.getTelescopicPosition()) || 
    (speed > 0 && Constants.Climber.kTelescopicFullExtend <= climber.getTelescopicPosition());
  }
}
