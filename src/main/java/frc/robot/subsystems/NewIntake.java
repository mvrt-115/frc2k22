// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.StatusFrame; // to set the status frame period of diff motors

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.TalonFactory;

public class NewIntake extends SubsystemBase {
  
  public static enum IntakeState {INTAKING, PIVOTING_DOWN, PIVOTING_UP, UP};
  
  private IntakeState state;

  private BaseTalon intakeMotor; 
  public BaseTalon pivotMotor; 

  private NewIntake() {
    //Set motors
    intakeMotor = TalonFactory.createTalonFX(Constants.Intake.kROLLER_ID, false);
    pivotMotor = TalonFactory.createTalonFX(Constants.Intake.kPIVOT_ID, true);
    
    //Other settings
    intakeMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 150); //Time period when sending info
    pivotMotor.setSelectedSensorPosition(0);
    pivotMotor.setNeutralMode(NeutralMode.Brake); //Brake here

    //PID
    pivotMotor.config_kP(Constants.kPIDIdx, Constants.Intake.kP);
    pivotMotor.config_kI(Constants.kPIDIdx, Constants.Intake.kI);
    pivotMotor.config_kD(Constants.kPIDIdx, Constants.Intake.kD);
    pivotMotor.config_kF(Constants.kPIDIdx, Constants.Intake.kFF);

    //Set state
    state = IntakeState.UP;
  }

  @Override
  public void periodic() {
    //Push with brake
    pivotMotor.set(ControlMode.PercentOutput, -0.5);
    pivotMotor.setNeutralMode(NeutralMode.Brake);



  }

  public void startIntake(){
    intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.kWHEELS_SPEED);
  }

  public void stopIntake(){
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public void stopPivot()
  {
    if(state == IntakeState.INTAKING){
      pivotMotor.set(ControlMode.PercentOutput, Constants.Intake.kPIVOT_STOP_SPEED_WHEN_DOWN);

    } else if(state == IntakeState.UP){
      pivotMotor.set(ControlMode.PercentOutput, Constants.Intake.kPIVOT_STOP_SPEED_WHEN_UP);
    
    } else {
      pivotMotor.set(ControlMode.PercentOutput, -0.2);
    }
  }

  public void pivotDown(){
    //Start intaking once pivoted down
    if(isAtBottom()){
      state = IntakeState.INTAKING;
    
    //Pivot down
    } else {
      pivotMotor.set(ControlMode.Position, Constants.Intake.kTICKS_TO_BOTTOM, DemandType.ArbitraryFeedForward, 0);
    }
  }

  public boolean isAtBottom()
  {
    return Math.abs(Constants.Intake.kTICKS_TO_BOTTOM - getCurrentPos()) <= Constants.Intake.kMARGIN_OF_ERROR_TICKS;
  }

  public double getCurrentPos()
  {
    return pivotMotor.getSelectedSensorPosition();
  }

}