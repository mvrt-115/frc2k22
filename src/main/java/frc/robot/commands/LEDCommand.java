// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LedState;

public class LEDCommand extends CommandBase {
  /** Creates a new LEDCommand. */
  public LEDs led;

  public enum GeneralStates {TELESCOPIC_RUNNING, SHOOTER_RUNNING, STORAGE};
  public GeneralStates currState = GeneralStates.STORAGE;

  public int numBalls = 1;
  public int lastBalls;
  public Turret turret;
  public Shooter shooter;
  public Climber climber;
  public Storage storage;
  public Intake intake;

  public LEDCommand(LEDs leds, Turret turret, Shooter shooter, Climber climber, Storage storage, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(leds);
    led = leds;
    this.turret = turret;
    this.shooter = shooter;
    this.climber = climber;
    this.storage = storage;
    this.intake = intake;
    lastBalls = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    numBalls = storage.getBalls();

    if(currState == GeneralStates.TELESCOPIC_RUNNING/*climber.leftTelescopic.getMotorOutputPercent() != 0*/)
      led.setCurrentState(LedState.CLIMBER);

    led.setTurretLEDs(shooter.getCurrentRPM(), shooter.getRequiredRPM(), turret.getTurretState());
    //led.setTurretLEDs(0, 7000, TurretState.DISABLED);

    if(led.getCurrentState() == LedState.CLIMBER) {
      if(led.getPreviousState() != LedState.CLIMBER)
      {
        led.setRainbow(0, led.LED_LENGTH-1);
        led.setPreviousState(LedState.CLIMBER);
      }
      
      led.moveDown(0, led.LED_LENGTH-1, 0.075);
    }

    else if(led.getCurrentState() == LedState.DEFAULT) {
      if(numBalls != lastBalls)
      {
        led.setMultiBlock(7, new Color[]{led.kMVRTPurple, led.kMVRTGold}, 0, led.LED_LENGTH/2 - (led.LED_LENGTH/4 * (2 - numBalls)));
        led.setSingleBlock(led.LED_LENGTH/2 - (led.LED_LENGTH/4 * (2 - numBalls)), led.LED_LENGTH/2, led.kBlack);
        led.setMultiBlock(7, new Color[]{led.kMVRTPurple, led.kMVRTGold}, led.LED_LENGTH/2 + (led.LED_LENGTH/4 * (2 - numBalls)), led.LED_LENGTH);
        led.setSingleBlock(led.LED_LENGTH/2, led.LED_LENGTH/2 - (led.LED_LENGTH/4 * (2 - numBalls)), led.kBlack);
        led.setPreviousState(LedState.DEFAULT);
      }

      led.moveDown(0, led.LED_LENGTH/2 - (led.LED_LENGTH/4 * (2 - numBalls)) - 1, 0.01);
      led.moveUp(led.LED_LENGTH/2 + (led.LED_LENGTH/4 * (2 - numBalls)), led.LED_LENGTH - 1, 0.01);
    }

    lastBalls = numBalls;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
