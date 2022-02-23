// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TalonFactory;

public class Storage extends SubsystemBase {
  /** Creates a new Storage. */
  private DigitalInput breakBeam; // break beam 
  private BaseTalon storageMotor; // motor that runs the belt
  public static enum StorageState {EXPELLING, NOT_EXPELLING}; // states of the storage as to whether it will expel the balls or not
  public static StorageState currentState; // the current state of the storage
  
  public Storage() {
    storageMotor = TalonFactory.createTalonSRX(Constants.Storage.kMotorID, true);
    breakBeam = new DigitalInput(Constants.Storage.kBreakBeamPort);
    currentState = StorageState.NOT_EXPELLING;
  }

  /**
   * periodically, it runs the motors when the break beam is broken. when the ball
   * goes out of the path of the break beam, the motor stops. this method allows
   * for both balls to be intaken and stored in the storage area.
   */
  @Override
  public void periodic() {
    if(currentState == StorageState.NOT_EXPELLING)
    {
      if(!breakBeam.get()){ // when the break beam is broken
        runMotor(false);
      }
      else{
       stopMotor();
      }
    }
  }

  /**
   * This runs the motor at a set speed to take in the balls into the storage container.
   * if a negative number is passed in, all the balls are expelled.
   * @param expel       Expel the ball if true
   */
  public void runMotor(boolean expel){
    storageMotor.set(ControlMode.PercentOutput, Constants.Storage.kMotorSpeed * (expel ? -1 : 1)); //smol if statement
  }

  /**
   * This stops the motor
   */
  public void stopMotor(){
    storageMotor.set(ControlMode.PercentOutput, 0);
  }
}
