// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Climber.Auton;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbMidRungAutonSequence extends SequentialCommandGroup {
  public Climber climber;
  /** Creates a new sequence for only the mid rung climb. */
  public ClimbMidRungAutonSequence(Climber climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.climber = climber;

    addCommands(
      // new ClimberAuton(climber, climber.pivot, Auton.kPivotPivotingBack),
      new ClimberAuton(climber, climber.leftTelescopic, Constants.Climber.Auton.kHookHighRungTele, climber.leftTelescopicProximity),
      new ClimberAuton(climber, climber.leftTelescopic, Constants.Climber.kTelescopicFullRetract),
      // new ClimberAuton(climber, climber.pivot, Auton.kRotateToHighRungPivot, climber.pivotLimit),
      new ClimberAuton(climber, climber.leftTelescopic, Auton.kExtendPivotHang, climber.pivotProximity)
    );
  }
}
