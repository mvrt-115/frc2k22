// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TalonFactory;

public class Storage extends SubsystemBase  {
  private DigitalInput breakbeamTop, breakbeamBott;
  private boolean prevStateTop, prevStateBott;
  private double lastTopChanged;
  private int balls;
  private BaseTalon motor;
  private boolean overriden;
  private double lastTime;
  private boolean readyShoot = false;
  private boolean intaking = false;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  public final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private double stopIntakeTime = -1;


  public Storage()  {
    breakbeamTop = new DigitalInput(1);
    breakbeamBott = new DigitalInput(2);
    prevStateTop = prevStateBott = true; // true is unbroken
    motor = TalonFactory.createTalonSRX(39, true);
    balls = -1;
    overriden = true;
    lastTime = Timer.getFPGATimestamp();
    lastTopChanged = Timer.getFPGATimestamp();
    // intaking = false;
  }

  public boolean isTopBreakbeamBroken() {
    return !breakbeamTop.get();
  }

  public void setIntaking(boolean intaking) {
    if(!intaking)
      stopIntakeTime = Timer.getFPGATimestamp();
    else
      intaking = true;
  }

  @Override
  public void periodic()  {
    runMotor(0.0);
    SmartDashboard.putBoolean("top breakbeam broken", !breakbeamTop.get());
    SmartDashboard.putBoolean("bottom breakbeam broken", !breakbeamBott.get());
    SmartDashboard.putNumber("number of balls", balls);
    SmartDashboard.putBoolean("overriden", overriden);
    SmartDashboard.putString("Ball color", getBallColor());

   
    if(prevStateBott && !breakbeamBott.get() && Timer.getFPGATimestamp() - lastTopChanged > 0.5) {
      balls++;
      lastTopChanged = Timer.getFPGATimestamp();
    }

    else if(!prevStateTop && breakbeamTop.get()&& Timer.getFPGATimestamp() - lastTime > 0.5)  {
      balls--;
      lastTime = Timer.getFPGATimestamp();
    }

    if(stopIntakeTime != -1 && Timer.getFPGATimestamp() - stopIntakeTime > 0.5) {
      intaking = false;
      stopIntakeTime = -1;
    }

    if(balls < 0) balls = 0;

    if(balls >= 3) balls = 2;

    
    prevStateBott = breakbeamBott.get();
    prevStateTop = breakbeamTop.get();
    if(!overriden)  {
      autoStorage();
    }
  }

  public void setOverriden(boolean state) {
    overriden = state;
  }

  public void autoStorage()  {
    switch (balls) {
      // when there is one ball run until it passes first breakbeam
      case 1:
        if(!breakbeamBott.get())
          runMotor(0.6);
        else 
          runMotor(0);
        break;
      case 2:
        if(breakbeamTop.get())
          runMotor(0.6);
        else 
          runMotor(0);
        break;
      case 0: 
        // if(intaking)
          runMotor(0.6);
        // else
        //   runMotor(0);
      default:
        break;
    }
  }



  public void runMotor(double out) {
    motor.set(ControlMode.PercentOutput, out);
  }

  public int getBalls() { return balls; }
  public void setBalls(int balls) { this.balls = balls; }

  public void setReadyShoot(boolean newShooting){
    readyShoot = newShooting;
  }
  public boolean getShooting() {
    return readyShoot;
  }
  public String getBallColor(){
    if(colorSensor.isConnected() && !breakbeamTop.get()){
     return getColor(colorSensor.getBlue(), colorSensor.getRed());
    }
    return "No Ball";

    }

    public String getColor( double blue, double red){
      if(colorSensor.getBlue() > colorSensor.getRed())
        return "Blue";
      else if(colorSensor.getRed() > colorSensor.getBlue())
        return "Red";
      return "No Ball";
    }
  }
  

