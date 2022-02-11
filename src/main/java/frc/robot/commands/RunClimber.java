// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunClimber extends SequentialCommandGroup {
  public Climber climber;
  /** Creates a new RunClimber. */
  public RunClimber(Climber climberIn) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    climber = climberIn;
    /** the climber auton which is designed for one complete climb is called
     *  twice to account for two climbs in order to get to the high rung */ 
    addCommands(
      new ClimberAutonSequence(climber),
      new ClimberAutonSequence(climber)
    );
  }
}
