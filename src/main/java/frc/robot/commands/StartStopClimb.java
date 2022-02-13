// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StartStopClimb extends ParallelRaceGroup {
  private Climber climber;
  private Supplier<Boolean> stopManualButton;
  /** Creates a new StartStopClimb. */
  public StartStopClimb(Supplier<Boolean> stopManualButton, Climber climber) {
    this.climber = climber;
    this.stopManualButton = stopManualButton;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    /**if the start button is called then this command will run the climb sequence but if 
     * the stop button is pressed, the stop command should be run even if the run climber
     * command didn't finish */ 
    addCommands(
      new RunClimber(climber),
      new StopClimb(climber, stopManualButton)
    );
  }
}
