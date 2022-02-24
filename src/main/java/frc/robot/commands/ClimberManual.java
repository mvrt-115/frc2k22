// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.subsystems.Climber;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.ClimberState;

public class ClimberManual extends CommandBase {
    public Climber climber;
    public TalonFX motor;
    public double speed;
    public Supplier<Boolean> button;
    /** Creates a new ClimberManual. */
    public ClimberManual(Climber climber, TalonFX motor, Supplier<Boolean> buttonState, double speed) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.motor = motor;
        this.speed = speed;
        this.button = buttonState;
        this.climber = climber;
        addRequirements(climber); 
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      climber.setSpeed(motor, speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      climber.stopMotor(motor);
    }

    // Returns true when the command should end.
    /* checks to see if the button has been released and the arms based on whichever button is being released, those arms
     * have not reached their limits whether it be the full extract/retract for the telescopic arms and the full pivot
     * forward/backward for the pivot arms. 
     */
    @Override
    public boolean isFinished() {
        return !button.get() && ((motor.equals(climber.leftTelescopic) 
        && ( speed < 0 && Constants.Climber.kTelescopicFullRetract >= climber.getTelescopicPosition()) 
        || (speed > 0 && Constants.Climber.kTelescopicFullExtend <= climber.getTelescopicPosition())) 
        || /*( motor.equals(climber.pivot)*/(speed < 0 && Constants.Climber.kPivotMaxForwardPos >= climber.getPivotAngle()) 
        || (speed > 0 && Constants.Climber.kPivotMaxReversePos <= climber.getPivotAngle()));
    }
}
