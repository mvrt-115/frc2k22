// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeCam extends SubsystemBase {
  // Camera Info:
  public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(4.5);
  public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(21);
  // Angle between horizontal and the camera.
  public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(-20);
  // How far from the target we want to be
  public static final double GOAL_RANGE_METERS = Units.feetToMeters(1);

  // Change this to match the name of your camera
  PhotonCamera camera = new PhotonCamera("gloworm");
  public IntakeCam() {
    // if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
    //   camera.setPipelineIndex(1);
    // } else {
    //   camera.setPipelineIndex(2);
    // }
      
    }    


  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
}
