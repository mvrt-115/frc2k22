// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Storage.StorageState;

public class ExpelBalls extends CommandBase {
  private Storage storage;
  private int times;
  private boolean inEndCommand = false;
  /** Creates a new ExpelBalls. */
  public ExpelBalls(Storage storageIn) {
    // Use addRequirements() here to declare subsystem dependencies.
    storage = storageIn;
    times = 0;
    addRequirements(storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    times++;
    inEndCommand = false;
    SmartDashboard.putNumber("num times command called", times);
    storage.setState(StorageState.EXPELLING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    inEndCommand = false;

    //storage.runMotor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    inEndCommand = true;
    SmartDashboard.putBoolean("command is ending", inEndCommand);
    storage.setState(StorageState.NOT_EXPELLING);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return storage.getBreakBeamLast().get() && storage.getBreakBeamFirst().get();
  }
}
