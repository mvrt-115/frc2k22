// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TalonFactory;

public class Storage extends SubsystemBase  {
  private DigitalInput breakbeamTop, breakbeamBott;
  private boolean prevStateTop, prevStateBott;
  private int balls;
  private BaseTalon motor;
  private boolean overriden;

  public Storage()  {
    breakbeamTop = new DigitalInput(0);
    breakbeamBott = new DigitalInput(1);
    prevStateTop = prevStateBott = true; // true is unbroken
    motor = TalonFactory.createTalonSRX(0, true);
    balls = 0;
    overriden = false;
  }

  @Override
  public void periodic()  {
    if(prevStateBott && !breakbeamBott.get()) {
      balls++;
    }

    else if(!prevStateTop && breakbeamTop.get())  {
      balls--;
    }

    prevStateBott = breakbeamBott.get();
    prevStateTop = breakbeamTop.get();
    if(!overriden)  {
      runMotor();
    }
  }

  public void setOverriden(boolean state) {
    overriden = state;
  }

  public void runMotor()  {
    if(balls == 1)  {
      if(!breakbeamBott.get())
        motor.set(ControlMode.PercentOutput, 0.5);
    }

    else if(balls == 2) {
      if(breakbeamTop.get())  {
        motor.set(ControlMode.PercentOutput, 0.5);
      }
    }

    else if(balls == 3) {
      motor.set(ControlMode.PercentOutput, 0.5);
    }
  }

  public BaseTalon getMotor(){ return motor; }

  public int getBalls() { return balls; }
}
