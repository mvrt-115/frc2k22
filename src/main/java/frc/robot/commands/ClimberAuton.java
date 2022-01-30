// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html


public class ClimberAuton extends SequentialCommandGroup {
  /** Creates a new ClimberAuto. */

  public ClimberAuton(Climber climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    final double distToGrabNextHook = 0.0; // should be simple math

    addCommands( // have to finish this and finalize movement stuff

      // initial extension already done as it is done via button presses

      new PivotArmAuton(climber, 0.0), // initial rotation position - slightly back
      new TelescopicArmAuton(climber, 0.0), // origianl retract to completely hook onto rung/bring robot off ground 
      new PivotArmAuton(climber, 0.0), // limit switch and then check with proximity sensor
      new TelescopicArmAuton(climber, 0.0),
      new PivotArmAuton(climber, 0.0),
      new TelescopicArmAuton(climber, 0.0),
      new PivotArmAuton(climber, 0.0), // telescopic arm moves, limit switch to check if it has contacted the high rung
      new TelescopicArmAuton(climber, 0.0), // retract arm and check proximity sensor and pivot should be off hook
      new PivotArmAuton(climber, distToGrabNextHook), // rotate the pivoting arm back
      new TelescopicArmAuton(climber, 0.0), // retract 
      new PivotArmAuton(climber, 0.0), // will go forward until limit switch is pressed
      new TelescopicArmAuton(climber, 0.0) // retract until proximity sensor of pivoting arm senses rung
      
      // redo the entire motion again
    );
  }
}
