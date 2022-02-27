// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Climber.Auton;
import frc.robot.commands.pivot.PivotAuton;
import frc.robot.commands.telescopic.TelescopicAuton;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html


public class TraversalAuton extends SequentialCommandGroup {
  /** Creates a new ClimberAuton. */
  public TraversalAuton(Climber climber) {

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

    //TelscopicAutons with the robot on mid rung
     new TelescopicAuton(climber, Auton.kLiftOffRungTele), 
     new PivotAuton(climber, Auton.kPivotTeleBack), 
     new TelescopicAuton(climber, Constants.Climber.kTelescopicFullExtend), 
     new TelescopicAuton(climber, Auton.kRotateToHighRungTele, climber.leftTelescopicLimit),
     new TelescopicAuton(climber, Auton.kHookHighRungTele, climber.leftTelescopicProximity),
     new PivotAuton(climber, Auton.kShiftWeight), 
     new TelescopicAuton(climber, Auton.kRetractPivotLiftOff), 
     new PivotAuton(climber, Auton.kPivotPivotingBack), 
     new TelescopicAuton(climber, Constants.Climber.kTelescopicFullRetract), 
     new PivotAuton(climber, Auton.kRotateToHighRungPivot, climber.pivotLimit), 
     new TelescopicAuton(climber,Auton.kExtendPivotHang, climber.pivotProximity) 
    );
  }
}

