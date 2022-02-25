// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class Forward extends SequentialCommandGroup {
  /** Creates a new Forward. */
  private Drivetrain drivetrain;

  public Forward(Drivetrain dr) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = dr;

    addCommands(
      drivetrain.getRamseteCommand(PathPlanner.loadPath("Forward", 3, 3))
    );
  }


  // public Command goForward()
  // {
  //   Trajectory trajectory = PathPlanner.loadPath("Forward", 3, 3);
  //   return drivetrain.getRamseteCommand(trajectory);
  // }

  // // Called when the command is initially scheduled.
  // @Override
  // public void initialize() {
    
  // }

  // // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {
  //   SmartDashboard.putBoolean("RUnning", true);
  // }

  // // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {
  //   SmartDashboard.putBoolean("RUnning", false);
  // }

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   // return drivetrain.getPose().getX() >= 3;
  //   return false;
  // }
}
