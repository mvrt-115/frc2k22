// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TalonFactory;

public class Climber extends SubsystemBase {

    public TalonFX leftTelescopic, rightTelescopic; // motors for each telescopic arm, controlling extending and
                                                    // collapsing motions
    public Servo leftServo, rightServo; // servos that act as ratchets
    public DigitalInput leftTelescopicProximity, rightTelescopicProximity; // inductive proximity
                                                                                           // sensors
    public boolean lowClimb; // check to see if the climb should be low or not                                                                                    
    // for detecting whether robot is hooked on rungs or not for each type of arm
    // public DigitalInput leftTelescopicLimit, rightTelescopicLimit; // inductive proximity sensors
    // for detecting whether robot is hooked on rungs or not for each type of arm

    private static Climber climber = new Climber();
    
    public static Climber getInstance() {
        return climber;
    }

    public enum ClimberState {
        NONE, TELESCOPIC_LIMIT, TELESCOPIC_PROXIMITY
        // proximity state is triggerd when both limit switch and proximity sensors are
        // triggered
  }

  

    private ClimberState telescopicState = ClimberState.NONE;

    /**
     * Initializes all objects and reconfigures all motors to requirements
     */

    private Climber() {
        // initializes all motors and sensors
        leftTelescopic = TalonFactory.createTalonFX(Constants.Climber.kLeftTelescopicID, false);
        rightTelescopic = TalonFactory.createTalonFX(Constants.Climber.kRightTelescopicID, false);
        leftTelescopic.setNeutralMode(NeutralMode.Coast);
        rightTelescopic.setNeutralMode(NeutralMode.Coast);
        resetTelescopicEncoder();

        leftTelescopic.setInverted(TalonFXInvertType.Clockwise);
        rightTelescopic.setInverted(TalonFXInvertType.Clockwise);

        leftServo = new Servo(Constants.Climber.kLeftServoID);
        rightServo = new Servo(Constants.Climber.kRightServoID); 

        setServoTurn(0);


        //reconfiguring all motors with PID constants        

        leftTelescopic.config_kP(Constants.kPIDIdx, Constants.Climber.kTelekP);
        leftTelescopic.config_kI(Constants.kPIDIdx, Constants.Climber.kTelekI);
        leftTelescopic.config_kD(Constants.kPIDIdx, Constants.Climber.kTelekD);
        leftTelescopic.config_kF(Constants.kPIDIdx, Constants.Climber.kTelekF);

        rightTelescopic.config_kP(Constants.kPIDIdx, Constants.Climber.kTelekP);
        rightTelescopic.config_kI(Constants.kPIDIdx, Constants.Climber.kTelekI);
        rightTelescopic.config_kD(Constants.kPIDIdx, Constants.Climber.kTelekD);
        rightTelescopic.config_kF(Constants.kPIDIdx, Constants.Climber.kTelekF);

        lowClimb = false;
    }

    /**
     * Sets speed to given motor
     * 
     * @param motor the motor that needs to be run
     * @param speed speed at which the motor needs to be run
     */
    public void setTelescopicSpeed(double speed) {
        if(speed == 0) {
            leftTelescopic.set(ControlMode.PercentOutput, 0); // test and change value
            rightTelescopic.set(ControlMode.PercentOutput, 0); 
            return;
        }
        if(!lowClimb && ((speed < 0 && getTelescopicPosition() <= 2000) || (speed > 0 && leftTelescopic.getSelectedSensorPosition() >= 276675))) { 
            leftTelescopic.set(ControlMode.PercentOutput, 0); // test and change value
            // rightTelescopic.set(ControlMode.PercentOutput, 0); 
        }
        else if(lowClimb && ((speed < 0 && getTelescopicPosition() <= 2000) || (speed > 0 && leftTelescopic.getSelectedSensorPosition() >= 168329))) {
            leftTelescopic.set(ControlMode.PercentOutput, 0); 
        }
        else {
            leftTelescopic.set(ControlMode.PercentOutput, speed);
        }

        if(!lowClimb && ((speed < 0 && rightTelescopic.getSelectedSensorPosition() <= 2000) || (speed > 0 && rightTelescopic.getSelectedSensorPosition() >= 271943))) {
            rightTelescopic.set(ControlMode.PercentOutput, 0); // test and change value
            // rightTelescopic.set(ControlMode.PercentOutput, 0); 
        }
        else if(lowClimb && ((speed < 0 && rightTelescopic.getSelectedSensorPosition() <= 2000) || (speed > 0 && rightTelescopic.getSelectedSensorPosition() >= 154098))) {
            rightTelescopic.set(ControlMode.PercentOutput, 0);
        }
        else {
            rightTelescopic.set(ControlMode.PercentOutput, speed);
        }

        // low climb soft limits
        /*if(lowClimb []\ 
        \][
            []\
            ][\
            ]
        ]]opic.set(ControlMode.PercentOutput, 0); // test and change value
            // rightTelescopic.set(ControlMode.PercentOutput, 0); 
        }
        else {
            leftTelescopic.set(ControlMode.PercentOutput, speed);
        }
        if(lowClimb && (speed < 0 && rightTelescopic.getSelectedSensorPosition() <= 2000) || (speed > 0 && rightTelescopic.getSelectedSensorPosition() >= )) { // change
            rightTelescopic.set(ControlMode.PercentOutput, 0); // test and change value
            // rightTelescopic.set(ControlMode.PercentOutput, 0); 
        }
        else {
            rightTelescopic.set(ControlMode.PercentOutput, speed);
        }*/
    }

    public void setClimb(boolean climb) {
        lowClimb = climb;
    }

    /**
     * Stops motors
     * 
     * @param motor the motor that needs to be stopped
     */
    public void stopTelescopicMotor() {
        setTelescopicSpeed(0);
    }
    
    /**
     * Sets position of telescopic motor
     * 
     * @param finalPosition final position of the motor
     * @param feedForward The feed forward
     
    public void setTelescopicPosition(double finalPosition, double feedForward) {
        leftTelescopic.set(ControlMode.Position, finalPosition, DemandType.ArbitraryFeedForward, feedForward);
    }*/

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
     * Gives the position of the telescopic arms through values given by encoder
     * 
     * @return The ticks the encoders have rotated
     */
    public double getTelescopicPosition() {
        double average = (getLeftTelescopicEncoderValue()) / 1; //left is a lot more firctionaly, so just use right
        return average;
    }

    public void resetTelescopicEncoder()
    {
        leftTelescopic.setSelectedSensorPosition(0);
        rightTelescopic.setSelectedSensorPosition(0);
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

    public void setServoTurn(double turn) {
        if(turn == 0 ){
            leftServo.set(turn);
            rightServo.set(turn);
            return;
        }
        //leftServo.set(turn);
       // rightServo.set(1);
        leftServo.setSpeed(0.8);
        rightServo.setSpeed(0.8);
    }

    public double getServoAngle() {
        return leftServo.getPosition();
    }

    public double getOffsetServoAngle() {
        return rightServo.getPosition();
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
         SmartDashboard.putNumber("left climber pos", getLeftTelescopicEncoderValue());
         SmartDashboard.putNumber("right climber pos", getRightTelescopicEncoderValue());
    }
}