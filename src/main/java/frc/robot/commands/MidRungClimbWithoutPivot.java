// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MidRungClimbWithoutPivot extends SequentialCommandGroup {
  /** Creates a new MidRungClimb. */
  public Climber climber;

  public MidRungClimbWithoutPivot(Climber climber) {
    this.climber = climber;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ClimberAuton(climber, climber.leftTelescopic, Constants.Climber.Auton.kHookHighRungTele, climber.leftTelescopicProximity),
      new ClimberAuton(climber, climber.leftTelescopic, Constants.Climber.kTelescopicFullRetract)
    );
  }
}
