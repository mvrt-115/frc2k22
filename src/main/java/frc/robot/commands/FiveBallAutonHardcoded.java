// I hope this works

package frc.robot.commands;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class FiveBallAutonHardcoded extends SequentialCommandGroup {
  
  private Drivetrain drivetrain;

  public FiveBallAutonHardcoded(Drivetrain dr, Intake intake) {
    drivetrain = dr;
    drivetrain.setOdometry(new Pose2d(0, 0, new Rotation2d(0)));

    addCommands(
      path6part1()
      //new IntakeBalls(intake),
      //new ShootBalls(shooter),
      //can we add the rotation2d thing here? set the angle as a parameter idk
      //how else do we add it as a command??
      // quickTurn(7.70, 0.68, 100),
      // path6part2(),
      // //new IntakeBalls(intake),
      // //new ShootBalls(shooter),
      // //drivetrain.quickTurn(35.7),
      // quickTurn(5.33, 1.83, 35.7),
      // path6part3()
      // //new IntakeBalls(intake)
      //new ShootBalls(shooter)
    );
  }

  public Command path6part1()
  {
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(8.09, 2.01, new Rotation2d(0)), 
      List.of(),
      new Pose2d(7.70, 0.68, new Rotation2d(0)), 
      new TrajectoryConfig(3, 4) 
    );

    // Transform2d transform2d = new Pose2d(8.09, 2.01, new Rotation2d(0)).minus(trajectory.getInitialPose());
    // trajectory = trajectory.transformBy(transform2d);

    drivetrain.setOdometry(trajectory.getInitialPose());
    return drivetrain.getRamseteCommand(trajectory);
  }

  public Command quickTurn(double x, double y, double angle) //idk if will work
  {
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(x, y, new Rotation2d(drivetrain.getGyroAngle().getDegrees())), 
      List.of( new Translation2d(x, y) ), //same spot
      new Pose2d(x, y, new Rotation2d(drivetrain.getGyroAngle().getDegrees() + angle)),
      new TrajectoryConfig(3, 4)
    );

    return drivetrain.getRamseteCommand(trajectory);
  }

  public Command path6part2()
  {
    Trajectory trajectory = PathPlanner.loadPath("Path6Part2", 4, 4);
    Transform2d transform2d = new Pose2d(7.70, 0.68, new Rotation2d(0)).minus(trajectory.getInitialPose());
    trajectory = trajectory.transformBy(transform2d);
    return drivetrain.getRamseteCommand(trajectory);
  }

  public Command path6part3()
  {
    Trajectory trajectory = PathPlanner.loadPath("Path6Part3", 4, 4);
    Transform2d transform2d = new Pose2d(5.33, 1.83, new Rotation2d(0)).minus(trajectory.getInitialPose());
    trajectory = trajectory.transformBy(transform2d);
    return drivetrain.getRamseteCommand(trajectory);
  }

  // // Called when the command is initially scheduled.
  // @Override
  // public void initialize() {
  //   SmartDashboard.putString("Auton Path 6", "Starting!");
  // }

  // // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {
  //   SmartDashboard.putString("Auton Path 6", "Running!");
  // }

  // // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {}

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return false;
  // }
}
