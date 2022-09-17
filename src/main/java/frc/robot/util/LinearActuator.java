// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.Point;

import frc.robot.Constants;

public class LinearActuator extends Servo {
  double max_length;
  double min_servo_length = 114.3;
  double horiz_length = 241.3;
  double radial_length = 203.2;

  double targetPos;
  double curPos;

  double lastTime = 0;
  double cosine_equation_constant = Math.pow(horiz_length, 2) + Math.pow(radial_length, 2);

  public LinearActuator(int channel, double length) {
    super(channel);
    setBounds(2, 1.8, 1.5, 1.2, 1);
    max_length = length;
  }

  /**
   * @param angle the angle of the hood in degrees
   */
  public void setPositionFromAngle(double angle) {

    double distance = Math
        .sqrt(cosine_equation_constant - 2 * horiz_length * radial_length * Math.cos(Math.toRadians(angle)))
        - min_servo_length;

    setPosition(distance);
  }

  public void setPosition(double servoPosition) {
    if (servoPosition < 0) {
      servoPosition = 0;
    } else if (servoPosition > max_length) {
      servoPosition = max_length;
    }

    SmartDashboard.putNumber("Servo Percent Output: ", servoPosition / max_length);
    set(servoPosition / max_length);
  }

  public double getPosition() {
    return curPos;
  }

  public boolean isFinished() {
    return curPos == targetPos;
  }

  public double getHoodAngle() {
    double length = getPosition() * max_length;
    double angle = Math.acos(Math.pow(cosine_equation_constant, 2) + Math.pow(241.3, 2)
        - Math.pow(max_length, 2) / (2 * cosine_equation_constant * 241.3));

    return Math.toDegrees(angle);
  }
}