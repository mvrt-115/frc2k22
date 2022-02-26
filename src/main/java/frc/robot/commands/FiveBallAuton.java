// I hope this works

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class FiveBallAuton extends SequentialCommandGroup {
  
  private Drivetrain drivetrain;
  private Intake intake;

  public FiveBallAuton(Drivetrain dr, Intake in) {
    drivetrain = dr;
    intake = in;

    drivetrain.setOdometry(new Pose2d(0, 0, new Rotation2d(0)));

    new ParallelCommandGroup(
      // new IntakeBalls(intake),
      new SequentialCommandGroup(
        runPath("Path6Path1"),
        //new ShootBalls(), //2 balls
        runPath("Path6Part2"),
        //new ShootBalls(), //1 ball
        runPath("Path6Part3")
        //new ShootBalls()  //2 balls
      )
    );
  }

  public Command runPath(String pathName)
  {
    Trajectory trajectory = PathPlanner.loadPath(pathName, 1, 1);
    drivetrain.setOdometry(trajectory.getInitialPose());
    return drivetrain.getRamseteCommand(trajectory);
  }

  /*
  @Override
  public void initialize() {
    SmartDashboard.putString("Auton Path 6", "Starting!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("Auton Path 6", "Running!");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  */
}
