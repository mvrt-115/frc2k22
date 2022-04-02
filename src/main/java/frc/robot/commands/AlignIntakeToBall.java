// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Drivetrain;

public class AlignIntakeToBall extends CommandBase {
  /** Creates a new AlignIntakeToBall. */

  private boolean notStopping;
  private Drivetrain drivetrain;

  // Camera Info:
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(21);
  final double TARGET_HEIGHT_METERS = Units.inchesToMeters(4.5);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(-20);
  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(1);

  // Change this to match the name of your camera
  PhotonCamera camera = new PhotonCamera("gloworm");

  //create PID controller
  PIDController throttlePID = new PIDController(0.9, 0.0, 0.0);
  PIDController turnPID = new PIDController(0.008, 0.0, 0.0);
  Storage storage;
  int prevballs;
  
  public AlignIntakeToBall(Drivetrain drivetrain2, boolean _notStopping) 
  {
    drivetrain = drivetrain2;
    notStopping = _notStopping;
    addRequirements(drivetrain);
    
  }
  public AlignIntakeToBall(Drivetrain drivetrain2, Storage str) 
  {
    drivetrain = drivetrain2;
    storage = str;
    prevballs = storage.getBalls();
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if(DriverStation.getAlliance() == DriverStation.Alliance.Red){

      // camera.setPipelineIndex(1);
    // }
    // else{
    //   camera.setPipelineIndex(4);
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = 0;
    double turn = 0;

    var results = camera.getLatestResult();
    if (results.hasTargets()) {
      // SmartDashboard.putBoolean("Target", true);
      double range = PhotonUtils.calculateDistanceToTargetMeters(
                      CAMERA_HEIGHT_METERS,
                      TARGET_HEIGHT_METERS,
                      CAMERA_PITCH_RADIANS,
                      Units.degreesToRadians(results.getBestTarget().getPitch()));
      // SmartDashboard.putNumber("Range", range);
      forward = -0.2;//throttlePID.calculate(range, GOAL_RANGE_METERS);
      turn = turnPID.calculate(results.getBestTarget().getYaw(), 0);
      
    }
    else{
      // SmartDashboard.putBoolean("Target", false);
    }
    // SmartDashboard.putNumber("for", forward);
    // SmartDashboard.putNumber("turn", turn);
    
    if(Math.abs(turn) <= 0.05){
      drivetrain.setDrivetrainMotorSpeed(-forward, -forward);
    }
    else {

      drivetrain.setDrivetrainMotorSpeed(-forward-turn, -forward+turn);
    }
    // SmartDashboard.putNumber("Forward", forward);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setDrivetrainMotorSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}