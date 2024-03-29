// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.StatusFrame; 
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TalonFactory;

public class Storage extends SubsystemBase  {
  private DigitalInput breakbeamTop, breakbeamBott;
  private boolean lowerBeamState, upperBeamState; // where true means the breakbeams are broken, and false means they are not broken
  private boolean lastLowerState, lastUpperState; 
  private int balls;
  private BaseTalon bottomMotor, topMotor;
  private boolean overriden;
  private double topOutput = -0.3;
  private double botOutput = -0.5;
  boolean ready = false;
  private static Storage storage = new Storage();

  public static Storage getInstance()
  {
    if(storage == null) 
    {
      storage = new Storage();
    }
    return storage;
  }

  private Storage() 
  {
    breakbeamTop = new DigitalInput(1);
    breakbeamBott = new DigitalInput(2);
    lowerBeamState = upperBeamState = true;
    lastLowerState = lastUpperState = true;
    balls = 1; //assumption that the storage is preloaded with one ball
    bottomMotor = TalonFactory.createTalonFX(11, false);
    topMotor = TalonFactory.createTalonFX(0, false);
    overriden = false;
  }

  @Override
  public void periodic()  
  {log();
    lastLowerState = lowerBeamState;
    lastUpperState = upperBeamState;

    lowerBeamState = breakbeamBott.get();
    upperBeamState = breakbeamTop.get();

    // if(lastLowerState && !lowerBeamState) balls++;
    // if(!lastUpperState && upperBeamState) balls--;
    if(ready) {
        topMotor.set(ControlMode.PercentOutput, -1);
        bottomMotor.set(ControlMode.PercentOutput, -1);

        return;
      }
    if(!overriden){
      
      
      if(upperBeamState == false) topMotor.set(ControlMode.PercentOutput,0);
      else  topMotor.set(ControlMode.PercentOutput, topOutput);

      if(lowerBeamState == false && upperBeamState == false) bottomMotor.set(ControlMode.PercentOutput, 0);
      else bottomMotor.set(ControlMode.PercentOutput, botOutput);

      
    }
    
  }


  public int getBalls()
  {
    return balls;
  }
  public void setBalls(int num){
    balls = num;
  }

  public void setOverriden(boolean ov) { overriden = ov; }
  
  public void log()
  {
    SmartDashboard.putNumber("balls", balls);
    SmartDashboard.putBoolean("top breakbeam broken", upperBeamState);
    SmartDashboard.putBoolean("bottom breakbeam broken", lowerBeamState);
    SmartDashboard.putNumber("topout", topMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("bottomout", bottomMotor.getMotorOutputPercent());
  }
  public void runMotor(double val){
    topMotor.set(ControlMode.PercentOutput, val);
    bottomMotor.set(ControlMode.PercentOutput, val);
  }
  public void setReadyShoot(boolean val){
    ready  = val;
  }
  public void setIntaking(boolean val){}
  /*private DigitalInput breakbeamTop, breakbeamBott;
  private boolean prevStateTop, prevStateBott;
  private double lastTopChanged;
  private int balls;
  private  BaseTalon motor;
  private boolean overriden;
  private double lastTime;
  private boolean readyShoot = false;
  private boolean intaking = true;
  // private final I2C.Port i2cPort = I2C.Port.kOnboard;
  // public final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private double stopIntakeTime = -1;
  private static Storage storage = new Storage();
  
  public static Storage getInstance() {
    return storage;
  }
  private Storage()  {
    breakbeamTop = new DigitalInput(1);
    breakbeamBott = new DigitalInput(3);
    prevStateTop = prevStateBott = true; // true is unbroken
    motor = TalonFactory.createTalonFX(39, false);
    motor.setStatusFramePeriod(StatusFrame.Status_1_General, 100); // updating status frame period
    motor.setControlFramePeriod(ControlFrame.Control_3_General, 100);
    balls = 0;
    overriden = false;
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
    SmartDashboard.putBoolean("top breakbeam broken", !breakbeamTop.get());
    SmartDashboard.putBoolean("bottom breakbeam broken", !breakbeamBott.get());
    SmartDashboard.putNumber("number of balls", balls);
    // SmartDashboard.putBoolean("overriden", overriden);
    // SmartDashboard.putString("Ball color", getBallColor());
    SmartDashboard.putNumber("Storage Output", motor.getMotorOutputPercent());
   
    if( prevStateBott && !breakbeamBott.get() && Timer.getFPGATimestamp() - lastTime > 0.25) {
      balls++;
      lastTime = Timer.getFPGATimestamp();
    }
    // else if(overriden && motor.getMotorOutputPercent() < 0 && !prevStateBott && breakbeamBott.get() && Timer.getFPGATimestamp() - lastTime > 0.3) {
    //   balls--;
    //   lastTime = Timer.getFPGATimestamp();
    // }
    else if(!prevStateTop && breakbeamTop.get())  {
      balls--;
      lastTopChanged = Timer.getFPGATimestamp();
    }
    if(stopIntakeTime != -1 && Timer.getFPGATimestamp() - stopIntakeTime > 0.3) {
      intaking = false;
      stopIntakeTime = -1;
    }
  
    if(balls < 0) balls = 0;
  
    if(balls >= 3) balls = 2;
      
    prevStateBott = breakbeamBott.get();
    prevStateTop = breakbeamTop.get();
      //if(!overriden)  {
    if(!readyShoot) autoStorage();
      //}
      //else{
       // runMotor(1);
     // }
     if (motor.hasResetOccurred()) {
       motor.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
     }
    
  }
  public void setOverriden(boolean state) {
    overriden = state;
  }
  public void autoStorage() {
    double s = 0.2;
    if(!breakbeamTop.get()) {
      runMotor(0);
      return;
    } 
      
    switch (balls) {
      // when there is one ball run until it passes first breakbeam
      case 1:
        if(!breakbeamBott.get())
          runMotor(s);
        else 
          runMotor(0);
        break;
      case 2:
        if(breakbeamTop.get())
          runMotor(s);
        else 
          runMotor(0);
        break;
      case 0: 
        // if(intaking)
           runMotor(s);
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
    System.out.println("BIG RACISM");
  }
  public boolean getShooting() {
    return readyShoot;
  }
  // public String getBallColor(){
  //   if(colorSensor.isConnected() && !breakbeamTop.get()){
  //    return getColor(colorSensor.getBlue(), colorSensor.getRed());
  //   }
  //   return "No Ball";
  //   }
  //   public String getColor( double blue, double red){
  //     if(colorSensor.getBlue() > colorSensor.getRed())
  //       return "Blue";
  //     else if(colorSensor.getRed() > colorSensor.getBlue())
  //       return "Red";
  //     return "No Ball";
  //   }
  }*/
}