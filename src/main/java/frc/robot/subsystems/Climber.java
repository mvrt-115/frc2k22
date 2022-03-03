// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MathUtils;
import frc.robot.util.TalonFactory;

public class Climber extends SubsystemBase {

    public TalonFX leftTelescopic, rightTelescopic, pivot; // motors for each telescopic arm, controlling extending and
                                                    // collapsing motions
    public Servo leftServo, rightServo; // servos that act as ratchets
    public DigitalInput leftTelescopicProximity, rightTelescopicProximity, pivotProximity; // inductive proximity
                                                                                           // sensors
    // for detecting whether robot is hooked on rungs or not for each type of arm
    public DigitalInput leftTelescopicLimit, rightTelescopicLimit, pivotLimit; // inductive proximity sensors
    // for detecting whether robot is hooked on rungs or not for each type of arm

    public enum ClimberState {
        NONE, TELESCOPIC_LIMIT, TELESCOPIC_PROXIMITY, PIVOT_LIMIT, PIVOT_PROXIMITY
        // proximity state is triggerd when both limit switch and proximity sensors are
        // triggered
    }

    // sets the telescopic climber state to nothing
    private ClimberState telescopicState = ClimberState.NONE;
   // private ClimberState pivotState = ClimberState.NONE;

    /**
     * Initializes all objects and reconfigures all motors to requirements
     */

    public Climber() {
        leftTelescopic = TalonFactory.createTalonFX(Constants.Climber.kLeftTelescopicID, false);
        rightTelescopic = TalonFactory.createTalonFX(Constants.Climber.kRightTelescopicID, false);

        leftTelescopic.setInverted(TalonFXInvertType.Clockwise);
        rightTelescopic.setInverted(TalonFXInvertType.CounterClockwise);

        leftServo = new Servo(Constants.Climber.kLeftServoID);
        rightServo = new Servo(Constants.Climber.kRightServoID); 

        leftTelescopicProximity = new DigitalInput(Constants.Climber.kLeftTelescopicProximityChannel);
        rightTelescopicProximity = new DigitalInput(Constants.Climber.kRightTelescopicProximityChannel);

        pivotProximity = new DigitalInput(Constants.Climber.kPivotProximityChannel);

        leftTelescopicLimit = new DigitalInput(Constants.Climber.kLeftTelescopicLimitSwitch);
        rightTelescopicLimit = new DigitalInput(Constants.Climber.kRightTelescopicLimitSwitch);

        pivotLimit = new DigitalInput(Constants.Climber.kPivotLimitSwitch);

        // reconfiguring all motors with PID constants
        rightTelescopic.follow(leftTelescopic);
        

        leftTelescopic.config_kP(Constants.kPIDIdx, Constants.Climber.kTelekP);
        leftTelescopic.config_kI(Constants.kPIDIdx, Constants.Climber.kTelekI);
        leftTelescopic.config_kD(Constants.kPIDIdx, Constants.Climber.kTelekD);
        leftTelescopic.config_kF(Constants.kPIDIdx, Constants.Climber.kTelekF);

        rightTelescopic.config_kP(Constants.kPIDIdx, Constants.Climber.kTelekP);
        rightTelescopic.config_kI(Constants.kPIDIdx, Constants.Climber.kTelekI);
        rightTelescopic.config_kD(Constants.kPIDIdx, Constants.Climber.kTelekD);
        rightTelescopic.config_kF(Constants.kPIDIdx, Constants.Climber.kTelekF);

        pivot.config_kP(Constants.kPIDIdx, Constants.Climber.kTelekP);
        pivot.config_kI(Constants.kPIDIdx, Constants.Climber.kTelekI);
        pivot.config_kD(Constants.kPIDIdx, Constants.Climber.kTelekD);
        pivot.config_kF(Constants.kPIDIdx, Constants.Climber.kTelekF);

        leftServo.setZeroLatch();
        rightServo.setZeroLatch();
    }

    /**
     * Sets speed to the telescopic motors
     * 
     * @param motor the motor that needs to be run
     * @param speed speed at which the motor needs to be run
     */
    public void setTelescopicSpeed(double speed) {
        leftTelescopic.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Sets speed to given motor
     * 
     * @param motor the motor that needs to be run
     * @param speed speed at which the motor needs to be run
     */
    public void setPivotSpeed(double speed) {
        leftTelescopic.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Stops telescopic motors
     * 
     * @param motor the motor that needs to be stopped
     */
    public void stopTelescopicMotor() {
        setTelescopicSpeed(0);
    }

    /**
     * Stops pivot motor
     * 
     * @param motor the motor that needs to be stopped
     */
    public void stopPivotMotor() {
        setPivotSpeed(0);
    }
    
    /**
     * Sets position of telescopic motor
     * 
     * @param finalPosition final position of the motor
     * @param feedForward The feed forward
     */
    public void setTelescopicPosition(double finalPosition, double feedForward) {
        leftTelescopic.set(ControlMode.Position, finalPosition, DemandType.ArbitraryFeedForward, feedForward);
    }

    /**
     * Sets position of pivot motor
     * 
     * @param finalPosition final position of the motor
     * @param feedForward The feed forward
     */
    public void setPivotMotor(double finalPosition, double feedForward) {
        pivot.set(ControlMode.Position, finalPosition, DemandType.ArbitraryFeedForward, feedForward);
    }

    /**
     * Get the encoder value of the left telescopic motors
     * 
     * @param motor the motor to get the value of
     * @return the number of ticks motor has rotated
     */
    public double getLeftTelescopicEncoderValue() {
        return leftTelescopic.getSelectedSensorPosition();
    }

    /**
     * Get the encoder value of the right telescopic motor
     * 
     * @return the number of ticks motor has rotated
     */
    public double getRightTelescopicEncoderValue() {
        return rightTelescopic.getSelectedSensorPosition();
    }

    /**
     * Get the encoder value of the pivot motor
     * 
     * @return the number of ticks motor has rotated
     */
    public double getPivotEncoderValue() {
        return pivot.getSelectedSensorPosition();
    }

    /**
     * Gives the position of the telescopic arms through values given by encoder
     * 
     * @return The inches the encoders have rotated
     */
    public double getTelescopicPosition() {
        double average = (getLeftTelescopicEncoderValue() + getRightTelescopicEncoderValue()) / 2;
        return MathUtils.ticksToInches(average);
    }

    /**
     * Get the state of the inductive proximity sensor on hook
     * 
     * @param limitSwitch proximity seonsor to get the state of
     * @return the state of proximity sensor (true/false)
     */
    public boolean getProximity(DigitalInput proximity) {
        return proximity.get();
    }

    /**
     * Get the state of the limit switch
     * 
     * @param limitSwitch limit switch to get the state of
     * @return the state of limit switch (true/false)
     */
    public boolean getLimitSwitch(DigitalInput limitSwitch) {
        return limitSwitch.get();
    }

    public boolean detectAllSensors(DigitalInput[] sensors) {
        for (DigitalInput sensor : sensors) {
            if (!sensor.get()) {
                return false;
            }
        }
        return true;
    }

    /* sets the telescopic state to the state given through the parameter
     * @param state of the climber at the moment 
     */
    public void setTelescopicState(ClimberState state) {
        telescopicState = state;
    }

    /* sets the pivot state to the state given through the parameter
     * @param state of the climber at the moment 
     
    public void setPivotState(ClimberState state) {
        pivotState = state;
    }*/

    /** stops all the motors */
    public void stopAllMotors() {
        stopTelescopicMotor();
    }

    /** gets the telescopic state of the climber 
     * @return telescopic state of the climber
     */
    public ClimberState getTelescopicState() {
        return telescopicState;
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
    //     // checks to see if the pivot limit switch has been contacted and then sets the state to the pivot limit state
    //     if(RobotContainer.PIVOT_EXISTS)
    //     if (getLimitSwitch(pivotLimit)) 
    //         setPivotState(ClimberState.PIVOT_LIMIT);
            
    //     // checks to see if the telescopic limit swtiches have been constacted and then sets the state to the telescopic limit state
    //     if (getLimitSwitch(rightTelescopicLimit) && getLimitSwitch(leftTelescopicLimit))
    //         setTelescopicState(ClimberState.TELESCOPIC_LIMIT);

    //     /* check to see if the telescopic proximity sense the rung and checks to make sure that telescopic limit swtiches are touched
    //      *  and then sets the telescopic state to the telescopic proximity (telescopic amrs on rung) */
    //     if (getProximity(leftTelescopicProximity) && getProximity(rightTelescopicProximity) && getPivotState() == ClimberState.TELESCOPIC_LIMIT) 
    //         setTelescopicState(ClimberState.TELESCOPIC_PROXIMITY);

    //     /* checks to see if the pivot proximity sense the rung and then checks to see if the previous state is set to the pivot limit switch
    //      * being contacted and then the pivot proximity state is set (pivot arms on rung) */
    //     if(RobotContainer.PIVOT_EXISTS && (pivotProximity.get()) && getPivotState() == ClimberState.PIVOT_LIMIT) 
    //       setPivotState(ClimberState.PIVOT_PROXIMITY);

    //     /* if the pivot proximity has not sensed the rung and the pivot state is the proximity state then the pivot state is set as none 
    //      * (pivot not on rung) */
    //     if(RobotContainer.PIVOT_EXISTS &&!getProximity(pivotProximity) && getPivotState() == ClimberState.PIVOT_PROXIMITY) 
    //         setPivotState(ClimberState.NONE);
            
    //     /* if the telescopic proximity sensors have not sensed the rung and the telescopic state is the proximity state then the 
    //      *  telescopic state is set to none (telescopic not on rung)
    //     */
    //     if(!getProximity(rightTelescopicProximity) &&!getProximity(leftTelescopicProximity) 
    //         && getPivotState() == ClimberState.TELESCOPIC_PROXIMITY) 
    //           setTelescopicState(ClimberState.NONE);

    //     if(Math.abs(leftTelescopic.getMotorOutputPercent()) <= Constants.Climber.kServoOnThreshold){
    //         leftServo.setAngle(Constants.Climber.kInitialServoAngle);
    //         rightServo.setAngle(Constants.Climber.kInitialServoAngle);
    //     }
    //     else {
    //         leftServo.setAngle(Constants.Climber.kServoTurn);
    //         rightServo.setAngle(Constants.Climber.kServoTurn);
    //     }
    }
}