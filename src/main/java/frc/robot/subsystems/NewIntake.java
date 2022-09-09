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

    switch(state){
      case INTAKING: // intake is deployed and starts running
        stopPivot();
        startIntake();
        break;

      case PIVOTING_UP: // intake goes back up and stops intaking  
        stopIntake();
        pivotUp();
        break;

      case PIVOTING_DOWN:
        pivotDown();
        break;

      case UP:
        //stopPivot(); // to keep the intake up
        pivotUp();
        stopIntake();
        break;
    }
  }

  //Log
  public void log() {
    SmartDashboard.putString("Intake State", state.toString());
    SmartDashboard.putNumber("Pivot Motor Voltage", pivotMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("Ticks", getCurrentPos());
    SmartDashboard.putBoolean("Is at top", Math.abs(getCurrentPos()) <= Constants.Intake.kMARGIN_OF_ERROR_TICKS);
    SmartDashboard.putBoolean("Is at bottom", Math.abs(getCurrentPos() - Constants.Intake.kTICKS_TO_BOTTOM) <= Constants.Intake.kMARGIN_OF_ERROR_TICKS);
  }

  //Start intaking
  public void startIntake(){
    intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.kWHEELS_SPEED);
  }

  //Stop intaking
  public void stopIntake(){
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  //Stop pivoting depending on state
  public void stopPivot(){
    if(state == IntakeState.INTAKING){
      pivotMotor.set(ControlMode.PercentOutput, Constants.Intake.kPIVOT_STOP_SPEED_WHEN_DOWN);

    } else if(state == IntakeState.UP){
      pivotMotor.set(ControlMode.PercentOutput, Constants.Intake.kPIVOT_STOP_SPEED_WHEN_UP);
    
    } else {
      pivotMotor.set(ControlMode.PercentOutput, -0.2);
    }
  }

  //Pivot down
  public void pivotDown(){
    //Start intaking once pivoted down
    if(isAtBottom()){
      state = IntakeState.INTAKING;
    
    //Pivot down
    } else {
      pivotMotor.set(ControlMode.Position, Constants.Intake.kTICKS_TO_BOTTOM, DemandType.ArbitraryFeedForward, 0);
    }
  }

  //Pivot up
  public void pivotUp(){
    if(isAtTop()){
      state = IntakeState.UP;
    
    } else {
      pivotMotor.set(ControlMode.Position, Constants.Intake.kTICKS_TO_TOP, DemandType.ArbitraryFeedForward, -0.1);
    }
  }

  public boolean isAtTop(){
    return Math.abs(getCurrentPos()) <= Constants.Intake.kMARGIN_OF_ERROR_TICKS;
  }

  public boolean isAtBottom(){
    return Math.abs(Constants.Intake.kTICKS_TO_BOTTOM - getCurrentPos()) <= Constants.Intake.kMARGIN_OF_ERROR_TICKS;

  }

  public double getCurrentPos(){
    return pivotMotor.getSelectedSensorPosition();

  }
}