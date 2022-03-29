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
import frc.robot.subsystems.Turret.TurretState;

public class FiveBallAuton extends ParallelCommandGroup {
  
  private Drivetrain drivetrain;
  Turret turret;

  public FiveBallAuton(Drivetrain dr, Intake in, Shooter shooter, Storage storage, Turret turret) {
    drivetrain = dr;
    this.turret = turret;

    storage.setBalls(1);

    turret.setState(TurretState.TARGETING);

    // drivetrain.setOdometry(new Pose2d(0, 0, new Rotation2d(0)));

    addCommands(
      new SequentialCommandGroup(
          new Pivot(in, storage),
          runPath("Path6Part1"),
          new PivotUp(in, storage),
          new SetRPM(shooter, storage, turret).withTimeout(2),
          // //need to intake here
          runPath("Path6Part1.5"), //going back
           //shoot 2 balls
          new Pivot(in, storage),
          runPath("Path6Part2"), //going to get other ball
          // //need to intake here
          new PivotUp(in, storage),
          new SetRPM(shooter, storage, turret).withTimeout(1), //shoot 1 ball
          new Pivot(in, storage),
          runPath("Path6Part3"), //going to terminal
          runPath("Path6Part4"), //going back to shoot
          new PivotUp(in, storage),
          new SetRPM(shooter, storage, turret).withTimeout(2) //shoot 2 balls
      ),
      new FindTarget(turret)
    );
    
    Trajectory trajectory = PathPlanner.loadPath("Path6Part1", 4.75, 3);
    drivetrain.setOdometry(trajectory.getInitialPose());
  }
  public Command runPath(String pathName)
  {
    boolean reversed = pathName.equals("Path6Part1.5") || pathName.equals("Path6Part4");
    double vel = 4.75;
    if(pathName.equals("Path6Part3"))
      vel = 1;

    Trajectory trajectory = PathPlanner.loadPath(pathName, vel, 3, reversed);
    return drivetrain.getRamseteCommand(trajectory);
  }
}
