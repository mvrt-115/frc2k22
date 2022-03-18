// I hope this works

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Turret;

public class TwoBallAuton extends CommandBase {
  
  private Drivetrain drivetrain;

  public TwoBallAuton(Drivetrain dr, Intake in, Shooter shooter, Storage storage, Turret turret) {
    drivetrain = dr; 
    addRequirements(drivetrain, in, shooter, storage, turret);
    // new RunDrive(drivetrain, in, storage).andThen(new SetRPM(shooter, storage, 3000)).schedule();

   
  }

 
}
