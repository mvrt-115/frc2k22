// I hope this works

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;

public class FiveBallAuton extends SequentialCommandGroup {
  
  private Drivetrain drivetrain;

  public FiveBallAuton(Drivetrain dr, Intake in, Shooter shooter, Storage storage) {
    drivetrain = dr;

    drivetrain.setOdometry(new Pose2d(0, 0, new Rotation2d(0)));

    addCommands(
      // new ParallelCommandGroup(
      // new IntakeBalls(intake),
        // new SequentialCommandGroup(
          runPath("Path6Part1"),
          //new WaitCommand(3),
          // runPath("Path6Part1EndTurn"),//.andThen(new WaitCommand(1)),
          //runPath("Path6Part1EndTurn"),
          new TurnDegrees(drivetrain, 180),
          new SetRPM(shooter, storage).withTimeout(4), // 2 balls
          new TurnDegrees(drivetrain, 180),
          runPath("Path6Part1.5"),
          runPath("Path6Part2"),
          // runPath("Path6Part2EndTurn"),//.andThen(new WaitCommand(1)),
          new TurnDegrees(drivetrain, 180),
          new SetRPM(shooter, storage).withTimeout(4), // 1 ball
          runPath("Path6Part3"),
          new WaitCommand(1),
          runPath("Path6Part4"),
          new SetRPM(shooter, storage).withTimeout(4)
          //new ShootBalls()  //2 balls
        // )
      // )
    );
    Trajectory trajectory = PathPlanner.loadPath("Path6Part1", 1, 1);
    drivetrain.setOdometry(trajectory.getInitialPose());
  }

  public Command runPath(String pathName)
  {
    boolean reversed = pathName.equals("Path6Part1.5");

    Trajectory trajectory = PathPlanner.loadPath(pathName, 1, 1, reversed);
      
    // System.out.println(trajectory);
    // System.out.println(drivetrain);
    // drivetrain.setOdometry(trajectory.getInitialPose());
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
