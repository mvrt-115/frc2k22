// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.fasterxml.jackson.databind.node.DoubleNode;
import com.fasterxml.jackson.databind.ser.std.StdArraySerializers.FloatArraySerializer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Turret.TurretState;

public class TurretManual extends CommandBase {
  // TODO: make relative? make field orientated?

  private Turret turret;

  private Supplier<Double> angle;
  private Supplier<Double> magnitude;

  private double turnAngle;
  private boolean turning;

  private TurretState prevState;

  /** 
   * Creates a new TurnTurret
   * @param turret The turret subsystem
   * @param angle The angle to turn; should be bounded by -180 to 180 degrees
   */
  public TurretManual(Turret turret, Supplier<Double> angle, Supplier<Double> magnitude) {
    this.turret = turret;

    this.angle = angle;
    this.magnitude = magnitude;

    turnAngle = 0;
    turning = false;

    prevState = turret.getState();

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(magnitude.get() >= 0.1) {
      prevState = turret.getState();

      turnAngle = angle.get();

      if(angle.get() > Constants.Turret.kMaxAngle)
        turnAngle = Constants.Turret.kMaxAngle;
      else if(angle.get() < Constants.Turret.kMinAngle)
        turnAngle = Constants.Turret.kMinAngle;

      SmartDashboard.putNumber("Turret Manual Degrees", turnAngle);

      turret.setState(TurretState.DISABLED);

      turret.turnToDegrees(turnAngle);

      turning = true;
    }

    if(turning && Math.abs(turret.getCurrentPositionDegrees() - turnAngle) <= 2) {
      turning = false;

      turret.setState(prevState);
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
