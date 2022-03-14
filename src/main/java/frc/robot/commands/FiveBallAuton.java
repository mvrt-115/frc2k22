// I hope this works

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Turret;

public class FiveBallAuton extends ParallelCommandGroup {
  
  private Drivetrain drivetrain;

  public FiveBallAuton(Drivetrain dr, Intake in, Shooter shooter, Storage storage, Turret turret) {
    drivetrain = dr;

    // drivetrain.setOdometry(new Pose2d(0, 0, new Rotation2d(0)));

    addCommands(
      new SequentialCommandGroup(
          runPath("Path6Part1"),
          new AlignIntakeToBall(drivetrain, true).withTimeout(2),
          //need to intake here
          new SetRPM(shooter, storage, turret).withTimeout(2), //shoot 2 balls
          runPath("Path6Part1.5"), //going back
          runPath("Path6Part2"), //going to get other ball
          new AlignIntakeToBall(drivetrain, true).withTimeout(2),
          //need to intake here
          new SetRPM(shooter, storage, turret).withTimeout(2), //intake 1 ball
          runPath("Path6Part3"), //going to terminal
          new AlignIntakeToBall(drivetrain, true).withTimeout(2),
          runPath("Path6Part4"), //going back to shoot
          new SetRPM(shooter, storage, turret).withTimeout(2) //shoot 2 balls
      ),
      new Pivot(in, storage)
    );
    
    Trajectory trajectory = PathPlanner.loadPath("Path6Part1", 1, 1);
    drivetrain.setOdometry(trajectory.getInitialPose());
  }

  public Command runPath(String pathName)
  {
    boolean reversed = pathName.equals("Path6Part1.5") || pathName.equals("Path6Part4");

    Trajectory trajectory = PathPlanner.loadPath(pathName, 1, 1, reversed);
    return drivetrain.getRamseteCommand(trajectory);
  }
}
