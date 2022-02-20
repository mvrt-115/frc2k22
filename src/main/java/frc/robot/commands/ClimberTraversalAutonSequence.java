// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Climber.Auton;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html


public class ClimberTraversalAutonSequence extends SequentialCommandGroup {
  /** Creates a new ClimberAuton. */
  public ClimberTraversalAutonSequence(Climber climber) {

    addCommands(

      // initial extension already done as it is done via button presses

      /*1. Extend telescopic arms
        2. Pivot forward
        5. Extend
        6. Pivot back with telescopic limit
        7. Retract with telescopic proximity 
        8. Retract small amount
        9. Pivot back
        10. Retract
        11. Pivot forward with pivot limit
        12. Extend with proximity*/

      // code begins with the robot on mid rung
     new ClimberAuton(climber, climber.leftTelescopic, Auton.kLiftOffRungTele), 
     new ClimberAuton(climber, climber.pivot, Auton.kPivotTeleBack), 
     new ClimberAuton(climber, climber.leftTelescopic, Constants.Climber.kTelescopicFullExtend), 
     new ClimberAuton(climber, climber.leftTelescopic, Auton.kRotateToHighRungTele, climber.leftTelescopicLimit),
     new ClimberAuton(climber, climber.leftTelescopic, Auton.kHookHighRungTele, climber.leftTelescopicProximity),
     new ClimberAuton(climber, climber.pivot, Auton.kShiftWeight), 
     new ClimberAuton(climber, climber.leftTelescopic, Auton.kRetractPivotLiftOff), 
     new ClimberAuton(climber, climber.pivot, Auton.kPivotPivotingBack), 
     new ClimberAuton(climber, climber.leftTelescopic, Constants.Climber.kTelescopicFullRetract), 
     new ClimberAuton(climber, climber.pivot, Auton.kRotateToHighRungPivot, climber.pivotLimit), 
     new ClimberAuton(climber, climber.leftTelescopic, Auton.kExtendPivotHang, climber.pivotProximity) 

      // code begins with the robot on mid rung
     /* new TelescopicArmAuton(climber, Auton.kLiftOffRungTele), // extend telescopic arms
      new PivotArmAuton(climber, Auton.kPivotTeleBack), // pivot telescopic arms back to get the telescopic behind the higher rung
      new TelescopicArmAuton(climber, Constants.Climber.kTelescopicFullExtend), // extend telescopic arms to get the telescopic limit switches lined up with the rung
      new TelescopicArmAutonLimit(climber, Auton.kRotateToHighRungTele), // pivots back for telescopic to touch rung at telescopic limit switch
      new TelescopicArmAutonProximity(climber, Auton.kHookHighRungTele), // hangs telescopic hooks onto the higher rung
      new PivotArmAuton(climber, Auton.kShiftWeight), // shift the weight onto the telescoping arm
      new TelescopicArmAuton(climber, Auton.kRetractPivotLiftOff), // retracts the telescopic arm to allow pivoting arm to rotate back
      new PivotArmAuton(climber, Auton.kPivotPivotingBack), // pivots pivoting arms back to get them behind the higher rung
      new TelescopicArmAuton(climber, Constants.Climber.kTelescopicFullRetract), // extend the telescopic arm to avoid pivoting arm from hitting the high run
      new PivotArmAutonLimit(climber, Auton.kRotateToHighRungPivot), // rotate the pivoting arm behind the high rung
      new PivotArmAutonProximity(climber, Auton.kRetractPivotHang) // retracts telescopic arms so that they hang on rung, and confirmed with proximities */
    );
  }
}

