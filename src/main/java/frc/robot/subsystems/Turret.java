// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Limelight;

public class Turret extends SubsystemBase {

  // 4090 ticks per rotation
  private TalonSRX motor;

  private Limelight limelight;

  /** Creates a new Turret. */
  public Turret(Limelight limelight) {
    this.limelight = limelight;

    motor = new TalonSRX(40);
    
    motor.configFactoryDefault();
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    motor.config_kP(0, Constants.Turret.kP);
    motor.config_kI(0, Constants.Turret.kI);
    motor.config_kD(0, Constants.Turret.kD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    
  }

  /**
   * Turns the turret the given amount of degrees
   * @param degrees The degrees to turn the motor (relative)
   */
  public void turn(double degrees) {
    motor.set(ControlMode.Position, degreesToTicks(degrees));
  }

  private int degreesToTicks(double degrees) {
    return (int) (((Constants.Turret.kEncoderTicks * Constants.Turret.kGearRatio) / 360) * degrees);
  }
}
