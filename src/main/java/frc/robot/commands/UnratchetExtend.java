// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.telescopic.TelescopicManual;
import frc.robot.commands.telescopic.TelescopicRatchet;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UnratchetExtend extends SequentialCommandGroup {
  public Climber climber;
  public Supplier<Boolean> buttonState;
  public double speed;
  /** Creates a new UnratchetExtend. */
  public UnratchetExtend(Climber climber, Supplier<Boolean> buttonState, double speed) {
    this.climber = climber;
    this.buttonState = buttonState;
    this.speed = speed;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // this just unratchets the arms and then allows the telescopic arms to extend
    addCommands(
      new TelescopicRatchet(climber, Constants.Climber.kServoUnRatchet).withTimeout(2),
      new TelescopicManual(climber, buttonState, speed)
    );
  }
}
