// I hope this works

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Turret.TurretState;

public class FourBallAuton extends ParallelCommandGroup {
  
  private Drivetrain drivetrain;
  Turret turret;

  public FourBallAuton(Drivetrain dr, Intake in, Shooter shooter, Storage storage, Turret turret) {
    drivetrain = dr;
    this.turret = turret;

    storage.setBalls(1);

    turret.setState(TurretState.TARGETING);

    addCommands(
      new SequentialCommandGroup(
          new Pivot(in, storage).alongWith(new WaitCommand(1.5)),
          runPath("FourBallPart1"),
          new PivotUp(in, storage).alongWith(new WaitCommand(1)),
          new SetRPM(shooter, storage, turret).withTimeout(3), //shooting 2 balls
          new Pivot(in, storage).alongWith(new WaitCommand(2)),
          runPath("FourBallPart2"), //going to terminal, intaking 1 ball + HP ball
          new WaitCommand(.5),
          new PivotUp(in, storage).alongWith(new WaitCommand(1)),
          runPath("FourBallPart3"), 
          new SetRPM(shooter, storage, turret).withTimeout(2), //shooting 2 balls
          new PivotUp(in, storage)
      ),
      new FindTarget(turret)
    );
    
  }

  @Override
  public void initialize() 
  {
    super.initialize();
    Trajectory trajectory = PathPlanner.loadPath("FourBallPart1", 4.75, 3);
    drivetrain.setOdometry(trajectory.getInitialPose());
  }

  public Command runPath(String pathName)
  {
    boolean reversed = pathName.equals("FourBallPart1") || pathName.equals("FourBallPart2");
    double vel = 2.75;
    Trajectory trajectory = PathPlanner.loadPath(pathName, vel, 2.5, reversed);
    return drivetrain.getRamseteCommand(trajectory);
  }

  public Command runPath(String pathName, double sp)
  {
    boolean reversed = pathName.equals("FourBallPart1") || pathName.equals("FourBallPart2");

    Trajectory trajectory = PathPlanner.loadPath(pathName, sp, 1.5, reversed);
    return drivetrain.getRamseteCommand(trajectory);
  }
}
