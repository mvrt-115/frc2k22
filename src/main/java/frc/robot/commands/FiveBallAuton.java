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
    turret.setState(TurretState.DISABLED);

    // drivetrain.setOdometry(new Pose2d(0, 0, new Rotation2d(0)));

    addCommands(
      new SequentialCommandGroup(
          new Pivot(in, storage),
          runPath("Path6Part1"),
          // new AlignIntakeToBall(drivetrain, true).withTimeout(1),
          // //need to intake here
          runPath("Path6Part1.5"), //going back
          new PivotUp(in, storage),
          new SetRPM(shooter, storage, turret).withTimeout(1.5), //shoot 2 balls
          new Pivot(in, storage),
          runPath("Path6Part2"), //going to get other ball
          new AlignIntakeToBall(drivetrain, storage).withTimeout(2),
          // //need to intake here
          new PivotUp(in, storage),
          new SetRPM(shooter, storage, turret).withTimeout(1.5), //shoot 1 ball
          new Pivot(in, storage),
          runPath("Path6Part3"), //going to terminal
          new AlignIntakeToBall(drivetrain, true).withTimeout(1.5),
          runPath("Path6Part4"), //going back to shoot
          new PivotUp(in, storage),
          new SetRPM(shooter, storage, turret).withTimeout(1.5) //shoot 2 balls
      ),
      new ZeroTurret(turret)
    );
    
    Trajectory trajectory = PathPlanner.loadPath("Path6Part1", 3, 1);
    drivetrain.setOdometry(trajectory.getInitialPose());
  }

  @Override
  public void end(boolean interrupted) {
      // TODO Auto-generated method stub
      super.end(interrupted);
      turret.setState(TurretState.TARGETING);
  }

  @Override
  public void execute() {
    // TODO Auto-generated method stub
    super.execute();
    turret.setState(TurretState.DISABLED);
  }

  public Command runPath(String pathName)
  {
    boolean reversed = pathName.equals("Path6Part1.5") || pathName.equals("Path6Part4");

    Trajectory trajectory = PathPlanner.loadPath(pathName, 1, 1, reversed);
    return drivetrain.getRamseteCommand(trajectory);
  }
}
