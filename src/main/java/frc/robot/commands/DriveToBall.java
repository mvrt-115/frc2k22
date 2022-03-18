// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.IntakeCam;

public class DriveToBall extends CommandBase {
  /** Creates a new DriveToBall. */
  PhotonCamera camera = new PhotonCamera("gloworm");
  Drivetrain drivetrain;
  public DriveToBall(Drivetrain dt) {
    drivetrain = dt;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var results = camera.getLatestResult();
    if (results.hasTargets()) {
      PhotonTrackedTarget cargo = results.getBestTarget();

      // Transform2d trans = cargo.getCameraToTarget();
      Translation2d trans = PhotonUtils.estimateCameraToTargetTranslation(
                                          PhotonUtils.calculateDistanceToTargetMeters(
                                                        IntakeCam.CAMERA_HEIGHT_METERS, 
                                                        IntakeCam.TARGET_HEIGHT_METERS, 
                                                        IntakeCam.CAMERA_PITCH_RADIANS, 
                                                        Units.degreesToRadians(results.getBestTarget().getPitch())), 
                                          Rotation2d.fromDegrees(cargo.getYaw())
      );

      Pose2d ballPose = new Pose2d(trans, drivetrain.getGyroAngle());


      
      // Trajectory pathToBall = new Trajectory(drivetrain.getPose(), list, ballPose, new TrajectoryConfig(3, 2));
      Trajectory pathToBall = TrajectoryGenerator.generateTrajectory(List.of(drivetrain.getPose(), ballPose), new TrajectoryConfig(3, 2));

      drivetrain.getRamseteCommand(pathToBall).schedule();
      
    }
    else{
    }
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
