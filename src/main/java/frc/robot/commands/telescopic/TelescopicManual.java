// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.telescopic;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class TelescopicManual extends CommandBase {
    public Climber climber;
    public double speed;
    public Supplier<Boolean> button;
    public boolean highlow_;
    /** Creates a new ClimberManual. */
    public TelescopicManual(Climber climber, Supplier<Boolean> buttonState, double speed, boolean highlow) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.speed = speed;
        this.button = buttonState;
        this.climber = climber;
        highlow_ = highlow;
        addRequirements(climber); 
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      climber.setClimb(highlow_);
    }

    // Called every time the scheduler runs while the command is scheduled.
    // sets the speed of the telescopic motors to the speed given 
    @Override
    public void execute() {
      climber.setTelescopicSpeed(speed);
    }

    // Called once the command ends or is interrupted.
    // stops the telescopic motors 
    @Override
    public void end(boolean interrupted) {
      climber.stopTelescopicMotor();
    }

    // Returns true when the command should end.
    /* checks to see if the button has been released  and that the arms have not 
     *  reached their limits whether it be the full extract/retract for the telescopic arms
     */
    @Override
    public boolean isFinished() {
      return !button.get() ;
        // return !button.get() || ((speed < 0 && climber.getTelescopicPosition() <= Constants.Climber.kTelescopicDownwardLimit) 
        // || (speed > 0 && climber.getTelescopicPosition() >= Constants.Climber.kTelescopicFullExtend));
    }
}
