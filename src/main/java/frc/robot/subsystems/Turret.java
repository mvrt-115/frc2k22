// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Limelight;
import frc.robot.util.Limelight.CAM_MODE;
import frc.robot.util.Limelight.LED_STATE;

public class Turret extends SubsystemBase {

  // 4090 ticks per rotation
  private TalonSRX motor;

  private Limelight limelight;

  /** Creates a new Turret. */
  public Turret(Limelight limelight) {
    this.limelight = limelight;

    motor = new TalonSRX(42);
    
    motor.configFactoryDefault();
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    motor.config_kP(0, Constants.Turret.kP);
    motor.config_kI(0, Constants.Turret.kI);
    motor.config_kD(0, Constants.Turret.kD);

    limelight.setLED(LED_STATE.ON);
    limelight.setPipeline(CAM_MODE.VISION_WIDE);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setDouble(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double angle =  NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    SmartDashboard.putNumber("tx", angle);

    // if(angle >= 10) {
    //   motor.config_kP(0, Constants.Turret.kPLarge);
    //   motor.config_kI(0, Constants.Turret.kILarge);
    //   motor.config_kD(0, Constants.Turret.kDLarge);
    // } else {
    //   motor.config_kP(0, Constants.Turret.kP);
    //   motor.config_kI(0, Constants.Turret.kI);
    //   motor.config_kD(0, Constants.Turret.kD);
    // }

    turn(angle);
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
