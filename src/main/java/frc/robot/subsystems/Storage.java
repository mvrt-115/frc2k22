// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TalonFactory;

public class Storage extends SubsystemBase {
  /** Creates a new Storage. */
  //private Ultrasonic ultrasonicFirst; // break beam
  private DigitalInput breakBeamFirst;
  private DigitalInput breakBeamLast; // change from AnalogInput to DigitalInput when
  //testing with breakbeam. Keep as AnalogInput when testing with UltrasonicSensor
  private BaseTalon storageMotor1; // motor that runs the belt
  public static enum StorageState {EXPELLING, NOT_EXPELLING}; // states of the storage as to whether it will expel the balls or not
  public static StorageState currentState; // the current state of the storage
  private int balls;
  public boolean firstBreakBeamBroken = false;
  public boolean secondBreakBeamBroken = false;
  
  // test the storage code! 
  // utilize the breakbeam placements from Wednesday
  // make sure states work
  
  public Storage() {
    storageMotor1 = TalonFactory.createTalonSRX(20, true);
    breakBeamLast = new DigitalInput(0);
    breakBeamFirst = new DigitalInput(1);
    currentState = StorageState.NOT_EXPELLING;
    balls = 0; // change on day of match
  }

  /**
   * periodically, it runs the motors when the break beam is broken. when the ball
   * goes out of the path of the break beam, the motor stops. this method allows
   * for both balls to be intaken and stored in the storage area.
   */
  @Override
  public void periodic() {
   // if(balls > 2) currentState = StorageState.EXPELLING;
    // uncomment above statement when testing on practice bot
    // test this later
   // if(balls == 0) currentState = StorageState.NOT_EXPELLING;
    
    if(currentState == StorageState.NOT_EXPELLING)
    {
      if(!breakBeamFirst.get())
      {
        SmartDashboard.putBoolean("running motor", true);
        runMotor();
      }

      else if(!breakBeamLast.get())
      {
        runMotor();
      }

      else if(breakBeamFirst.get() && breakBeamLast.get())
      {
        stopMotor();
        
      }

      if(breakBeamLast.get())
      {
        secondBreakBeamBroken = false;
      }

      if(breakBeamFirst.get())
      {
        firstBreakBeamBroken = false;
      }
    }

    else if(currentState == StorageState.EXPELLING)
    {
      runMotor();
    }

    SmartDashboard.putString("current state", getCurrentStateAsString());
    SmartDashboard.putNumber("number of balls in hopper", balls);
    SmartDashboard.putBoolean("is second breakbeam broken", !breakBeamLast.get());
    SmartDashboard.putBoolean("is first breakbeam broken", !breakBeamFirst.get());
  }

  private String getCurrentStateAsString()
  {
    switch(currentState){
      case NOT_EXPELLING: return "NOT EXPELLING";
      case EXPELLING: return "EXPELLING";
      default: return "";
    }
  }

  /**
   * Increments the ball variable
   */
  public void incrementBalls() {
    balls++;
  }
  /**
   * Decrements the ball variable
   */
  public void decrementBalls() {
    balls--;
  }
  /**
   * Returns the number of balls currently in the storage
   * @return the number of balls currently in storage
   */
  public int getBalls() {
    return balls;
  }

  public DigitalInput getBreakBeamFirst()
  {
    return breakBeamFirst;
  }

  public DigitalInput getBreakBeamLast()
  {
    return breakBeamLast;
  }
  /**
   * This runs the motor at a set speed to take in the balls into the storage container.
   * if a negative number is passed in, all the balls are expelled.
   * @param expel       Expel the ball if true
   */
  public void runMotor(){
    storageMotor1.set(ControlMode.PercentOutput, Constants.Storage.kMotorSpeed * (currentState == StorageState.EXPELLING ? -1 : 1)); //smol if statement

    if(currentState == StorageState.NOT_EXPELLING)
   {
      if(!firstBreakBeamBroken && !breakBeamFirst.get())
      {
     //   incrementBalls();
        firstBreakBeamBroken = true;
      }

      if(!secondBreakBeamBroken && !breakBeamLast.get())
      {
      //  decrementBalls();
        secondBreakBeamBroken = true;
      }
   }

   else if(currentState == StorageState.EXPELLING)
   {
     if(!breakBeamFirst.get() && !firstBreakBeamBroken)
     {
      firstBreakBeamBroken = true; 
     // decrementBalls();
     }
   }
  }

  public void setState(StorageState stateIn)
  {
    currentState = stateIn;
  }

  /**
   * This stops the motor
   */
  public void stopMotor(){
    storageMotor1.set(ControlMode.PercentOutput, 0);
  }
}
