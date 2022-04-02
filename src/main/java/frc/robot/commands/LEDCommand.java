// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.LEDs.LedState;

public class LEDCommand extends CommandBase {
  /** Creates a new LEDCommand. */
  public LEDs led;
  public Turret turret;
  public Shooter shooter;
  public Climber climber;
  public Storage storage;
  public Intake intake;

  public LEDCommand(LEDs leds, Turret turret, Shooter shooter, Climber climber, Storage storage, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    led = leds;
    this.turret = turret;
    this.shooter = shooter;
    this.climber = climber;
    this.storage = storage;
    this.intake = intake;

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(climber.leftTelescopic.getMotorOutputPercent() != 0 && Timer.getMatchTime() < 30)
      led.setCurrentState(LedState.CLIMBER);
    else if(Timer.getMatchTime() > 30)
      led.setCurrentState(LedState.DEFAULT);

    led.setTurretLEDs(shooter.getCurrentRPM(), shooter.getRequiredRPM(), turret.getTurretState());

    if(led.getCurrentState() == LedState.CLIMBER && led.getPreviousState() != LedState.CLIMBER) {
      led.setRainbow(0, led.LED_LENGTH-1);
      led.setPreviousState(LedState.CLIMBER);
      led.moveDown(0, led.LED_LENGTH-1, 0.1);
    }
    else if(led.getCurrentState() == LedState.DEFAULT && led.getPreviousState() != LedState.DEFAULT) {
      led.setWave(0, led.LED_LENGTH/2 - (led.LED_LENGTH/4 * (2 - storage.getBalls())), 7, 
        led.RBGToColor(new int[]{85, 5, 117}), led.RBGToColor(new int[]{255, 196, 16}));
      led.setWave(led.LED_LENGTH/2, led.LED_LENGTH - (led.LED_LENGTH/4 * (2 - storage.getBalls())), 7, 
        led.RBGToColor(new int[]{85, 5, 117}), led.RBGToColor(new int[]{255, 196, 16}));
      led.setPreviousState(LedState.DEFAULT);

      led.moveDown(0, led.LED_LENGTH/2 - (led.LED_LENGTH/4 * (2 - storage.getBalls())), 0.1);
      led.moveDown(led.LED_LENGTH/2, led.LED_LENGTH - (led.LED_LENGTH/4 * (2 - storage.getBalls())), 0.1);
    }
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
