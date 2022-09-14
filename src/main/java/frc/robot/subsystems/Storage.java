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
  private double output = -0.5;
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
  {
    lastLowerState = lowerBeamState;
    lastUpperState = upperBeamState;

    lowerBeamState = breakbeamBott.get();
    upperBeamState = breakbeamTop.get();

    if(lastLowerState && !lowerBeamState) balls++;
    if(!lastUpperState && upperBeamState) balls--;

    if(upperBeamState == false) topMotor.set(ControlMode.PercentOutput,0);
    else  topMotor.set(ControlMode.PercentOutput,output );

    if(lowerBeamState == false && upperBeamState == false) bottomMotor.set(ControlMode.PercentOutput, 0);
    else bottomMotor.set(ControlMode.PercentOutput, output);

    log();
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
  }
  public void runMotor(double val){

  }
  public void setReadyShoot(boolean val){}
  public void setIntaking(boolean val){}
  /*private DigitalInput breakbeamTop, breakbeamBott;
  private boolean prevStateTop, prevStateBott;
  private int balls;
  private BaseTalon motorTop;
  private BaseTalon motorBottom;
  private double lastTime;
  private boolean readyShoot = false;
  // private final I2C.Port i2cPort = I2C.Port.kOnboard;
  // public final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private double stopIntakeTime = -1;

  private static NewStorage storage = new NewStorage();
  
  public static NewStorage getInstance() {
    return storage;
  }


  private Storage()  {
    breakbeamTop = new DigitalInput(1);
    breakbeamBott = new DigitalInput(3);
    prevStateTop = prevStateBott = true; // true is unbroken
    
    //Set motors
    motorTop = TalonFactory.createTalonFX(39, false);
    motorTop.setStatusFramePeriod(StatusFrame.Status_1_General, 100); // updating status frame period
    motorTop.setControlFramePeriod(ControlFrame.Control_3_General, 100);
    motorBottom = TalonFactory.createTalonFX(39, false);
    motorBottom.setStatusFramePeriod(StatusFrame.Status_1_General, 100); // updating status frame period
    motorBottom.setControlFramePeriod(ControlFrame.Control_3_General, 100);

    balls = 0;
    lastTime = Timer.getFPGATimestamp();
  }

  //!!!! - assume both motors are running in periodic

    @Override
    public void periodic(){
        //Top broken
        if (!breakbeamTop.get()){
            //Stop top motor
            stopMotorTop();
        }

        //Both broken
        if(!breakbeamTop.get() && !breakbeamBott.get()){
            //Stop bottom
            stopMotorBottom();
        }

        //Top broken now 
        if(prevStateTop && !breakbeamTop.get()){
            //Add ball
            balls++;
        }

        //Top is broken and bottom is broken
        if (!breakbeamTop.get() && !breakbeamBott.get()){
            balls++;
        }

        if(balls < 0){
            balls = 0;
        }
    
        if(balls >= 3){
            balls = 2;
        }

        //Update states
        prevStateBott = breakbeamBott.get();
        prevStateTop = breakbeamTop.get();
        readyShoot = balls > 0;
    }

    public void runMotorTop(double out) {
        motorTop.set(ControlMode.PercentOutput, out);
    }

    public void runMotorBottom(double out) {
        motorBottom.set(ControlMode.PercentOutput, out);
    }

    public void stopMotorTop(){
        runMotorTop(0);
    }

<<<<<<< HEAD
    public void stopMotorBottom(){
        runMotorBottom(0);
    }
=======
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
  
