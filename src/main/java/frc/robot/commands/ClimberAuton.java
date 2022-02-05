// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html


public class ClimberAuton extends SequentialCommandGroup {
  /** Creates a new ClimberAuto. */

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

      new TelescopicArmAuton(climber, kLiftOffRungTele), // extend telescopic arms
      new PivotArmAuton(climber, 0.0), // pivot arms forward a little to get it behind mid rung
      new TelescopicArmAuton(climber, -0.0), // retract telescopic arms so that they are able to be low enough to go under rung
      new PivotArmAuton(climber, 0.0), // pivot arms forward to get the telescopic under the next rung
      new TelescopicArmAuton(climber, 0.0), // extend telescopic arms to get the telescopic limit switches lined up with the rung
      new PivotArmAutonLimit(climber, 0.0), // pivots back for telescopic to touch rung at telescopic limit switch
      new TelescopicArmAutonProximity(climber, -0.0), // retracts telescopic arms so that they hang on rung, and confirmed with proximities

      //TELESCOPIC ON RUNG AT THIS POINT

      new TelescopicArmAuton(climber, kLiftOff), // retracts further to remove hook from lower rung
      new PivotArmAuton(climber, -0.0), // pivots arm back towards the higher rung
      new TelescopicArmAuton(climber, -0.0), // retracts arm to level limit to higher rung
      new PivotArmAutonLimit(climber, 0.0), // pivots towards rung until limit is activated
      new PivotArmAutonProximity(climber, -0.0) // retracts arm to hook pivot arms to rung until proximities activate
    );
  }
}
