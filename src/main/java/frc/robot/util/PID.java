// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class PID {
    enum PIDType {
        POSITION, VELOCITY;
    }

    BaseTalon talon;
    PIDType type;

    double kp; // * (P)roportional Tuning Parameter
    double ki; // * (I)ntegral Tuning Parameter
    double kd; // * (D)erivative Tuning Parameter
    double kf; // * (F)eed Forward 

	int controllerDirection;

    double inputPrev;
    double outputSum;
    double setPoint;
    double error;

    double time;
    double outMin, outMax;

    /**
     * Create new PID object
     */
    public PID(BaseTalon talon, double Kp, double Ki, double Kd, double Kf, double Min, double Max, PIDType type)
    {
        setPID(Kp, Ki, Kd, Kf);
        this.talon = talon;
        setOutputLimits(Min, Max);
        this.type = type;
        this.inputPrev = 0;
    }

    /**
     * Set the PID target value (in ticks)
     * @param setPoint
     */
    public void setValue(double setPoint)
    {
        this.setPoint = setPoint;
        time = Timer.getFPGATimestamp()*1000;
    }

    public void setOutputLimits(double Min, double Max)
    {
        this.outMax = Max;
        this.outMin = Min;
    }

    public void setPID(double Kp, double Ki, double Kd, double Kf)
    {
        kp = Kp;
        ki = Ki;
        kd = Kd;
        kf = Kf;
    }

    public void computePeriodic()
    {
        double now = Timer.getFPGATimestamp()*1000;
        double deltaTime = now - time;
        time = now;
        double error = 0;
        double dInput = 0;
        double currVal = 0;
        switch (type)
        {
            case VELOCITY:
                currVal = talon.getSelectedSensorVelocity();
                break;
            case POSITION:
                currVal = talon.getSelectedSensorPosition();
                break;
        }
        error = setPoint - currVal;
        dInput = currVal - inputPrev;
        inputPrev = currVal;

        outputSum += (ki * error);

        if (outputSum > outMax) 
            outputSum= outMax;
        else if (outputSum < outMin) 
            outputSum= outMin;
        
        double output = kp * error;
        output += outputSum - kd * dInput / deltaTime;

        if (output > outMax) 
            output= outMax;
        else if (output < outMin) 
            output= outMin;
        
        switch (type)
        {
            case VELOCITY:
                talon.set(ControlMode.Velocity, output);
                break;
            case POSITION:
                talon.setSelectedSensorPosition(output);
                break;
        }
    }
}
