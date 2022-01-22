// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class PIDTune extends CommandBase {
  /** Creates a new PIDTune. */
  BaseTalon talon;
  String name;
  double P;
  double I;
  double D;
  double F;
  double rpm;

  public PIDTune(BaseTalon talon, double P, double I, double D, double F, String name) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.talon = talon;
    this.P = P;
    this.I = I;
    this.D = D;
    this.F = F;
    this.name = name;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber(name + " P", P);
    SmartDashboard.putNumber(name + " I", I);
    SmartDashboard.putNumber(name + " D", D);
    SmartDashboard.putNumber(name + " F", F);
    SmartDashboard.putBoolean("Changing " + name + " PID", true);
    rpm = talon.getSelectedSensorVelocity();
    talon.set(ControlMode.Velocity, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    P = SmartDashboard.getNumber(name + " P", P);
    I = SmartDashboard.getNumber(name + " I", I);
    D = SmartDashboard.getNumber(name + " D", D);
    F = SmartDashboard.getNumber(name + " F", F);
    talon.config_kP(Constants.kPIDIdx, P);
    talon.config_kI(Constants.kPIDIdx, I);
    talon.config_kD(Constants.kPIDIdx, D);
    talon.config_kF(Constants.kPIDIdx, F);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Changing " + name + " PID", false);
    talon.set(ControlMode.Velocity, rpm);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
