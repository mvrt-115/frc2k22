// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Climber.Auton;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html


public class ClimberAuton extends SequentialCommandGroup {
  /** Creates a new ClimberAuton. */
  public ClimberAuton(Climber climber) {

    addCommands(

      // initial extension already done as it is done via button presses

      /*new PivotArmAuton(climber, 0.0), // initial rotation position - slightly back
      new TelescopicArmAuton(climber, 0.0), // origianl retract to completely hook onto rung/bring robot off ground 
      new PivotArmAuton(climber, 0.0), // limit switch and then check with proximity sensor
      new TelescopicArmAuton(climber, 0.0),
      new PivotArmAuton(climber, 0.0),
      new TelescopicArmAuton(climber, 0.0),
      new PivotArmAuton(climber, 0.0), // telescopic arm moves, limit switch to check if it has contacted the high rung
      new TelescopicArmAuton(climber, 0.0), // retract arm and check proximity sensor and pivot should be off hook
      new PivotArmAuton(climber, distToGrabNextHook), // rotate the pivoting arm back
      new TelescopicArmAuton(climber, 0.0), // retract 
      new PivotArmAuton(climber, 0.0), // will go forward until limit switch is pressed
      new TelescopicArmAuton(climber, 0.0) // retract until proximity sensor of pivoting arm senses rung
      */
      // redo the entire motion again

      /*1. Extend telescopic arms
        2. Pivot forward
        3. Retract 
        4. Pivot further forward 
        5. Extend
        6. Pivot back with telescopic limit
        7. Retract with telescopic proximity 
        8. Retract small amount
        9. Pivot back
        10. Retract
        11. Pivot forward with pivot limit
        12. Extend with proximity*/

      // code begins with the robot on mid rung
      new TelescopicArmAuton(climber, Auton.kLiftOffRungTele), // extend telescopic arms
      new PivotArmAuton(climber, Auton.kFirstForwardPivot), // pivot arms forward a little to get it behind mid rung
      new TelescopicArmAuton(climber, Auton.kGetUnderRungRetract), // retract telescopic arms so that they are able to be low enough to go under rung
      new PivotArmAuton(climber, Auton.kSecondForwardPivot), // pivot arms forward to get the telescopic under the next rung
      new TelescopicArmAuton(climber, Auton.kTelescopicFullExtend), // extend telescopic arms to get the telescopic limit switches lined up with the rung
      new TelescopicArmAutonLimit(climber, Auton.kRotateToHighRung), // pivots back for telescopic to touch rung at telescopic limit switch
      new TelescopicArmAutonProximity(climber, Auton.kHookHighRung), 
      new PivotArmAuton(climber, Auton.kShiftWeight), // shift the weight on to the Telescoping arm
      new TelescopicArmAuton(climber, Auton.kRetractPivotLiftOff), // retracts the telescopic arm to allow pivoting arm to rotate back
      new PivotArmAuton(climber, Auton.kFirstBackPivot), // rotate the pivoting arm, ready to retract the telscopic arm
      new TelescopicArmAuton(climber, Auton.kExtendPivotingUnderRung), // retract telescopic arm to pivot pivoting arm 
      new PivotArmAuton(climber, Auton.kSecondBackPivot), // rotate the arm only a little to avoid hitting the high rung
      new TelescopicArmAuton(climber, Auton.kTelescopicFullRetract), // extend the telescopic arm to avoid pivoting arm from hitting the high run
      new PivotArmAutonLimit(climber, Auton.kRotateToHighRung), // rotate the pivoting arm behind the high rung
      new PivotArmAutonProximity(climber, Auton.kRetractPivotHang) // retracts telescopic arms so that they hang on rung, and confirmed with proximities
    );
  }
}
