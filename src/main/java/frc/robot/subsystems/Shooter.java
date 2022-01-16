// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Limelight;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private Limelight limelight;
  private double targetRPM = 0;
  private ShooterState currState;
  public enum ShooterState {
    OFF, SPINNINGUP, ATSPEED;
  }
  /** Creates a new Shooter. */
  public Shooter(Limelight limelight) {
    this.limelight = limelight;
  }

  

  public double ticksToRPM(double ticks)
  {
    return ticks * 600 / Constants.Shooter.TICKS_PER_REVOLUTION / Constants.Shooter.GEAR_RATIO;
  }

  public void setTargetRPM(double desiredVelocity) {
        targetRPM = Math.min(8000, desiredVelocity);
        
        if (desiredVelocity == 0)
            setShooterState(ShooterState.OFF);
        else{
            setShooterState(ShooterState.SPINNINGUP);
        }
    }

  public void stop()
  {

  }

  public void getRequiredRPM(){
    
  }

  public void setShooterState(ShooterState cs)
  {
    currState = cs;
    if(currState == ShooterState.OFF)
      targetRPM = 0;
    else if(currState == ShooterState.ATSPEED){

    }
      
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    switch(currState)
    {
      
    }
  }
}
