// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TalonFactory;

public class Storage extends SubsystemBase {
  /** Creates a new Storage. */
  private DigitalInput breakBeamFirst; // break beam 
  private BaseTalon storageMotor1, storageMotor2; // motor that runs the belt
  public static enum StorageState {EXPELLING, NOT_EXPELLING}; // states of the storage as to whether it will expel the balls or not
  public static StorageState currentState; // the current state of the storage
  public int balls; // 
  public DigitalInput breakBeamLast;
  public boolean firstBreakBeamBroken = false;
  public boolean secondBreakBeamBroken = false;
  
  public Storage() {
    storageMotor1 = TalonFactory.createTalonSRX(Constants.Storage.kMotor1ID, true);
    storageMotor2 = TalonFactory.createTalonSRX(Constants.Storage.kMotor2ID, true);
    breakBeamFirst = new DigitalInput(Constants.Storage.kBreakBeamPort0);
    breakBeamLast = new DigitalInput(Constants.Storage.kBreakBeamPort1);
    currentState = StorageState.NOT_EXPELLING;
    balls = 1; // change on day of match
  }

  /**
   * periodically, it runs the motors when the break beam is broken. when the ball
   * goes out of the path of the break beam, the motor stops. this method allows
   * for both balls to be intaken and stored in the storage area.
   */
  @Override
  public void periodic() {
    if(getBalls() >= 3) {
      currentState = StorageState.EXPELLING;
      runMotor();
    }

    if(currentState == StorageState.NOT_EXPELLING)
    {
      if(!breakBeamFirst.get()){ // when the break beam is broken
        runMotor();
        firstBreakBeamBroken = true;
      }
      else{
        if (firstBreakBeamBroken) {
          incrementBalls();
          firstBreakBeamBroken = !firstBreakBeamBroken;
        }
        stopMotor();
      }
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
  /**
   * This runs the motor at a set speed to take in the balls into the storage container.
   * if a negative number is passed in, all the balls are expelled.
   * @param expel       Expel the ball if true
   */
  public void runMotor(){
    storageMotor1.set(ControlMode.PercentOutput, Constants.Storage.kMotorSpeed * (currentState == StorageState.EXPELLING ? -1 : 1)); //smol if statement
    storageMotor2.set(ControlMode.PercentOutput, Constants.Storage.kMotorSpeed * (currentState == StorageState.EXPELLING ? -1 : 1)); //smol if statement

    if(!breakBeamLast.get()){
      secondBreakBeamBroken = true;
    }
    
    if(breakBeamLast.get()){
      if(secondBreakBeamBroken){
        decrementBalls();
        secondBreakBeamBroken = !(1 > 0);
      }
    }
  }

  /**
   * This stops the motor
   */
  public void stopMotor(){
    storageMotor1.set(ControlMode.PercentOutput, 0);
    storageMotor2.set(ControlMode.PercentOutput, 0);
  }
}