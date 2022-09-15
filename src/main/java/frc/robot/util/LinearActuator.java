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
  double m_speed;
  double m_length;

  double setPos;
  double curPos;

  double lastTime = 0;
  double dist = Math.sqrt(Math.pow(241.3,2) + Math.pow(203.2,2) - 2*241.3*203.2*Math.cos(Math.toRadians(30)));

  public LinearActuator(int channel, int length, double d) {
    super(channel);
    setBounds(2, 1.8, 1.5, 1.2, 1);
    m_length = length;
    m_speed = d;
  }

  public void setPositionFromAngle(double angle) {
    double horiz_length = 241.3;
    double vert_length = 203.2;

    dist = Math.sqrt(Math.pow(horiz_length,2) + Math.pow(vert_length,2) - 2*horiz_length*vert_length*Math.cos(Math.toRadians(angle)))- 114.3; 

    if(dist <= m_length && dist >= 0)
    {
      setPosition(dist);
    }

    else{
      setPosition(dist > m_length? m_length: 0);
    }
  }

  public void setPosition(double setPoint) {
    if(setPoint < 0) {
      setPoint = 0;
    }
    else if(setPoint > m_length) {
      setPoint = m_length;
    }

    SmartDashboard.putNumber("ho", setPoint / m_length);
    set(setPoint / m_length);
  }

  public void updateCurPos() {
    double dt = Timer.getFPGATimestamp() - lastTime;
    if(curPos > setPos + m_speed*dt) {
      curPos -= m_speed * dt;
    }
    else if(curPos < setPos-m_speed * dt){
      curPos += m_speed * dt;
    }
    else {
      curPos = setPos;
    }
    
  }

  public double getPosition() {
    return curPos;
  }

  public boolean isFinished() {
    return curPos == setPos;
  }

  public double getHoodAngle() {
    double length = getPosition() * m_length;
    double angle = Math.acos(Math.pow(dist, 2) + Math.pow(241.3, 2)-Math.pow(m_length, 2)/(2*dist*241.3));

    return Math.toDegrees(angle);
  }
}