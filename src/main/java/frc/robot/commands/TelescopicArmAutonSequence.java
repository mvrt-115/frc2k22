// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TelescopicArmAutonSequence extends SequentialCommandGroup {
  /** Creates a new TelescopicArm. */
  public TelescopicArmAutonSequence(Climber climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( // finish this
        new TelescopicArmAuton(climber, 0.0), // 0 is just a placement value for the actual distance that we would want our telescopic arm to
        new TelescopicArmAuton(climber, 0.0)// extend/retract to
    );
  }
}