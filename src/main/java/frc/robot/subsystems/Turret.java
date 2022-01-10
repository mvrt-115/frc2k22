// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Limelight;

public class Turret extends SubsystemBase {
  Limelight limelight;
  /** Creates a new Turret. */
  public Turret(Limelight limelight) {
    this.limelight = limelight;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
