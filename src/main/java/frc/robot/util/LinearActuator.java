// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import org.opencv.core.Point;

import frc.robot.Constants;

public class LinearActuator {
  BaseTalon actuatorMotor;
  double position;
  double radius;
  double distFromBase;
  double minHeight;
  double maxHeight;
  double degreesFromHorizontal;

  /** Creates a new LinearActuator. */
  public LinearActuator(BaseTalon actuatorMotor, double radius, double distFromBase, double height, double maxHeight, double degreesFromHorizontal) {
    this.actuatorMotor = actuatorMotor;
    position = 0;
    this.radius = radius;
    this.distFromBase = distFromBase;
    this.minHeight = height;
    this.maxHeight = maxHeight;
    this.degreesFromHorizontal = degreesFromHorizontal;

    this.actuatorMotor.config_kP(Constants.kPIDIdx, Constants.Actuator.P);
    this.actuatorMotor.config_kI(Constants.kPIDIdx, Constants.Actuator.I);
    this.actuatorMotor.config_kD(Constants.kPIDIdx, Constants.Actuator.D);
  }

  /**
   * Give an angle and radius and set actuator position to get that angle
   * @param angle (in degrees)
   */
  public void setPositionFromAngle(double angle)
  {
    double theta = Math.toRadians(angle + degreesFromHorizontal);
    double pos = Math.sqrt(Math.pow(radius, 2) + Math.pow(distFromBase, 2) - 2 * radius * distFromBase * Math.cos(theta));
    if (maxHeight - pos >= 0 && pos - minHeight >= 0)
      setPosition(pos);
    else
      setPosition(pos > maxHeight? maxHeight : minHeight);
  }

  /**
   * Sets the position given a parameter, if the given value is less than the height of the
   * unextended linear actuator, then the position is set to the base height
   * @param position
   */
  public void setPosition(double position)
  {
    if(position>=minHeight)
    {
      this.position = position;
      actuatorMotor.set(ControlMode.Position, positionToTicks(position));
    }
    else
    {
      this.position = minHeight;
    }
  }

  /**
   * Get motor controller ticks from linear actuator position
   * @param position
   * @return ticks
   */
  public double positionToTicks(double position)
  {
    return position / Constants.Actuator.THREAD_DISTANCE * Constants.Actuator.GEAR_RATIO * Constants.Actuator.TICKS_PER_ROTATION;
  }

  /**
   * Returns the angle (in degrees) of the hood from the position of the linear actuator
   * @return angle (in degrees)
   */
  public double getHoodAngle()
  {
    double length = minHeight+position;

    double angle = Math.acos(Math.pow(radius, 2)+Math.pow(distFromBase, 2)-Math.pow(length, 2)/(2*radius*distFromBase));
  
    return Math.toDegrees(angle);
  }

  /**
   * Returns the Talon object of the linear actuator
   * @return Talon
   */
  public BaseTalon getTalon()
  {
    return actuatorMotor;
  }
}
