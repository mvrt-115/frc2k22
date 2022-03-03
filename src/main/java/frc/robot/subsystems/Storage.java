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
import frc.robot.commands.SetRPM;
import frc.robot.util.TalonFactory;

public class Storage extends SubsystemBase  {
  private DigitalInput breakbeamTop, breakbeamBott;
  private boolean prevStateTop, prevStateBott;
  private double lastTopChanged;
  private int balls;
  private BaseTalon motor;
  private boolean overriden;
  private double lastTime;
  private Shooter shooter;
  private boolean readyShoot = false;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  public final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);


  public Storage(Shooter shooter)  {
    this.shooter = shooter;
    breakbeamTop = new DigitalInput(0);
    breakbeamBott = new DigitalInput(3);
    prevStateTop = prevStateBott = true; // true is unbroken
    motor = TalonFactory.createTalonSRX(39, true);
    balls = 1;
    overriden = false;
    lastTime = Timer.getFPGATimestamp();
    lastTopChanged = Timer.getFPGATimestamp();
  }

  @Override
  public void periodic()  {
    SmartDashboard.putBoolean("top breakbeam broken", !breakbeamTop.get());
    SmartDashboard.putBoolean("bottom breakbeam broken", !breakbeamBott.get());
    SmartDashboard.putNumber("number of balls", balls);
    SmartDashboard.putBoolean("overriden", overriden);
   
    if(prevStateBott && !breakbeamBott.get() && Timer.getFPGATimestamp() - lastTopChanged > 0.3) {
      balls++;
      lastTopChanged = Timer.getFPGATimestamp();
    }

    else if(!prevStateTop && breakbeamTop.get()&& Timer.getFPGATimestamp() - lastTime > 0.3)  {
      balls--;
      lastTime = Timer.getFPGATimestamp();
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
        motor.set(ControlMode.PercentOutput, 1);

      else motor.set(ControlMode.PercentOutput, 0);
    }

    else if(balls == 2) {
      if(breakbeamTop.get())  {
        motor.set(ControlMode.PercentOutput, 1);
      }
      else motor.set(ControlMode.PercentOutput, 0);
    }
    else if(balls>=3){
      new SetRPM(shooter, this, 50);
    }
    else if(balls == 0){
      // motor.set(ControlMode.PercentOutput, 1);
    }

    else{
      motor.set(ControlMode.PercentOutput,0);
    }
    // motor.set(ControlMode.PercentOutput, 1);

    
  }


  public BaseTalon getMotor(){ return motor; }

  public void runMotor(double out) {
    motor.set(ControlMode.PercentOutput, out);
  }

  public int getBalls() { return balls; }

  public void setReadyShoot(boolean newShooting){
    readyShoot = newShooting;
  }
  public boolean getShooting() {
    return readyShoot;
  }
  public String getBallColor(){
    if(colorSensor.isConnected() && colorSensor.getProximity() > 300){

      if(colorSensor.getBlue() > 2 * colorSensor.getRed())
        return "Blue";
      else if(colorSensor.getRed() > 2 * colorSensor.getBlue())
        return "Red";
      
      return "No Ball";
    }
    return "No Ball";

    }
  }
  

