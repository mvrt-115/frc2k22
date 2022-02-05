// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.TalonFactory;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  
  public static enum IntakeState {INTAKING, PIVOTING_DOWN, PIVOTING_UP, UP};
  private IntakeState state;

  // motors for the intake --> currently BaseTalon, may change 
  // (decide type of motor later)
  private BaseTalon intakeMotor; 
  private BaseTalon pivotMotor; 

  private double feedForward; // feed forward double needed to pivot for a certain number of ticks


  public Intake() {
    state = IntakeState.UP;

    intakeMotor = TalonFactory.createTalonSRX(Constants.Intake.kROLLER_ID, true); // change motor IDs from Constants later
    pivotMotor = TalonFactory.createTalonSRX(Constants.Intake.kPIVOT_ID, false); // change motor IDs from Constants later
   
    pivotMotor.setSelectedSensorPosition(0);

    pivotMotor.config_kP(Constants.kPIDIdx, Constants.Intake.kP);
    pivotMotor.config_kI(Constants.kPIDIdx, Constants.Intake.kI);
    pivotMotor.config_kD(Constants.kPIDIdx, Constants.Intake.kD);
    pivotMotor.config_kF(Constants.kPIDIdx, Constants.Intake.kFF);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    feedForward = Constants.Intake.kFF * Math.cos(Math.toRadians(getAngle()));
    switch(state)
    {
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
        stopPivot(); // to keep the intake up
        break;
    }

    SmartDashboard.putString("current state of intake", getCurrentStateAsString());
    SmartDashboard.putNumber("power voltage on pivot motor", pivotMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("Ticks", getCurrentPos());
    SmartDashboard.putBoolean("is at top", Math.abs(getCurrentPos()) <= Constants.Intake.kMARGIN_OF_ERROR_TICKS);
    SmartDashboard.putBoolean("is at bottom", Math.abs(getCurrentPos() - Constants.Intake.kTICKS_TO_BOTTOM) <= Constants.Intake.kMARGIN_OF_ERROR_TICKS);

  }
    
  public String getCurrentStateAsString()
  {
    switch(state)
    {
      case UP: return "Up";
      case PIVOTING_DOWN: return "Pivoting down";
      case PIVOTING_UP: return "pivoting up";
      case INTAKING: return "intaking";
      default: return "";
    }
  }
  
  /**
   * stops the motor which intakes the ball.
   */
  public void stopIntake()
  {
    intakeMotor.set(ControlMode.PercentOutput, 0);
   // uncomment when  intake motor is added
  }

  /**
   * stops the motor which pivots the intake.
   */
  public void stopPivot()
  {
    if(state == IntakeState.INTAKING)
      pivotMotor.set(ControlMode.PercentOutput, Constants.Intake.kPIVOT_STOP_SPEED_WHEN_DOWN);
    else if(state == IntakeState.UP)
      pivotMotor.set(ControlMode.PercentOutput, Constants.Intake.kPIVOT_STOP_SPEED_WHEN_UP);
    else 
      pivotMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * moves the intake down. if it is already at the bottom, then it sets
   * the state to intaking so that intaking can occur and stops the pivot
   * motor.
   * 
   * if it is not at the bottom, then it runs the pivot motor at a constant 
   * speed.
   */
  public void pivotDown()
  {
    if(isAtBottom())
    {
      state = IntakeState.INTAKING;
    }
    else
    {
      pivotMotor.set(ControlMode.Position, Constants.Intake.kTICKS_TO_BOTTOM, DemandType.ArbitraryFeedForward, 
      feedForward);
    }
  }

  public IntakeState getState()
  {
    return state;
  }

  public void setState(IntakeState stateIn)
  {
    state = stateIn;
  }

  /**
   * if the current position is within an acceptable range of the ticks it
   * takes to get to the bottom, then this method returns true.
   * 
   * @return  true if the intake is at the bottom when pivoting
   */
  public boolean isAtBottom()
  {
    return Math.abs(Constants.Intake.kTICKS_TO_BOTTOM - getCurrentPos()) <= Constants.Intake.kMARGIN_OF_ERROR_TICKS;
  }

  /**
   * if the current position is within an acceptable range of the ticks it
   * takes to get to the top, then this method returns true.
   * 
   * @return  true if the intake is at the top when pivoting
   */
  public boolean isAtTop()
  {
    return Math.abs(getCurrentPos()) <= Constants.Intake.kMARGIN_OF_ERROR_TICKS;
    // Math.abs(getCurrentPos() - Constants.Intake.kTicksTOTop <= blah blah)
  }

  /**
   * @return the current position of the intake in ticks.
   */
  public double getCurrentPos()
  {
    return pivotMotor.getSelectedSensorPosition();
  }

  /**
   * this method pivots the intake up by running the pivot motor at a 
   * constant speed.
   * 
   * if the intake is at the top, then it sets the pivot motor to a smaller
   * speed so that it allows the pivot to stay up, but not go any further
   */
  public void pivotUp()
  {
    if(isAtTop())
    {
      System.out.println("yo wassup guys im up");
      state = IntakeState.UP;
    }
    else
    {
      pivotMotor.set(ControlMode.Position, Constants.Intake.kTICKS_TO_TOP, DemandType.ArbitraryFeedForward, 
      feedForward);
    }
  }

  /**
   * runs the intake so that the wheels move, intaking the ball in
   */
  public void startIntake()
  {
    intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.kWHEELS_SPEED);
    // uncomment when intake motor is added
  }

  /**
   * @return The current angle of the pivot motor
   */
  public double getAngle() {
    return 90 + (getCurrentPos() / 1000 * 100);
  }
}
