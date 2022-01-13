// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import frc.robot.Constants;
import frc.robot.util.TalonFactory;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  
  public enum IntakeState {INTAKING, PIVOTING_UP, PIVOTING_DOWN, UP, DOWN};
  private IntakeState state;

  // motors for the intake --> currently BaseTalon, may change 
  // (decide type of motor later)
  private BaseTalon intakeMotor;
  private BaseTalon pivotMotor; // change if using piston and not motor

  private double feedForward;

  public Intake() {
    state = IntakeState.UP;

    intakeMotor = TalonFactory.createTalonSRX(Constants.Intake.kRollerId, false); // change motor IDs from Constants later
    pivotMotor = TalonFactory.createTalonSRX(Constants.Intake.kPivotId, false); // change motor IDs from Constants later

    pivotMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    feedForward = Constants.Intake.kFF * Math.cos(Math.toRadians(getAngle()));
    
    switch(state)
    {
      case INTAKING:
        stopPivot();
        startIntake();
        break;
      case PIVOTING_UP:
        stopIntake();
        pivotUp();
        break;
      case PIVOTING_DOWN:
        stopIntake();
        pivotDown();
        break;
      case UP:
        stopPivot();
        stopIntake();
        break;
      case DOWN:
        stopPivot();
        stopIntake();
        break;
    }
  }

  /**
   * stops the motor which intakes the ball.
   */
  public void stopIntake()
  {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * stops the motor which pivots the intake.
   */
  public void stopPivot()
  { 
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
      state = IntakeState.DOWN;
      stopPivot();
    }
    else
    {
      pivotMotor.set(ControlMode.Position, Constants.Intake.kTicksToBottom, DemandType.ArbitraryFeedForward, 
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
    return Math.abs(Constants.Intake.kTicksToBottom - getCurrentPos()) <= Constants.Intake.kMarginOfErrorTicks;
  }

  /**
   * if the current position is within an acceptable range of the ticks it
   * takes to get to the top, then this method returns true.
   * 
   * @return  true if the intake is at the top when pivoting
   */
  public boolean isAtTop()
  {
    return Math.abs(getCurrentPos() - Constants.Intake.kTicksToTop) <= Constants.Intake.kMarginOfErrorTicks;
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
      state = IntakeState.UP;
      stopPivot();
    }

    else
    {
      pivotMotor.set(ControlMode.Position, Constants.Intake.kTicksToTop, DemandType.ArbitraryFeedForward, 
      feedForward);
    }
  }

  /**
   * runs the intake so that the wheels move, intaking the ball in
   */
  public void startIntake()
  {
    intakeMotor.set(ControlMode.PercentOutput, Constants.Intake.kRollerOutput);
  }

  /**
   * converts the current position in ticks to angle
   * @return    the current angle of the intake
   */
  public double getAngle() {
    return 90 + (getCurrentPos() / 1000 * 100);
  }
}

