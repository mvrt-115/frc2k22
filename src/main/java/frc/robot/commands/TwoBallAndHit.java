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

public class TwoBallAndHit extends ParallelCommandGroup {
  
  private Drivetrain drivetrain;
  Turret turret;

  public TwoBallAndHit(Drivetrain dr, Intake in, Shooter shooter, Storage storage, Turret turret) {
    drivetrain = dr;
    this.turret = turret;

    storage.setBalls(1);

    turret.setState(TurretState.TARGETING);

    addCommands(
      new SequentialCommandGroup(
          new Pivot(in, storage).alongWith(new WaitCommand(1)),
          runPath("GoBack"),
          new PivotUp(in, storage).alongWith(new WaitCommand(1)),
          // new SetRPMRequired(shooter, storage, turret).withTimeout(3), //shooting 2 balls
          new SetRPMRequired(shooter, storage).withTimeout(3),
          new Pivot(in, storage).alongWith(new WaitCommand(1)),
          runPath("INTAKEBall"), //don't ask why this is capitalized
          new WaitCommand(.5),
          new PivotUp(in, storage).alongWith(new WaitCommand(1)),
          new ZeroTurret(turret).withTimeout(1),
          new SetRPMValue(shooter, storage, 1500).withTimeout(2), //shooting 2 balls
          new PivotUp(in, storage)
      )
    );
    
  }

  @Override
  public void initialize() 
  {
    super.initialize();
    Trajectory trajectory = PathPlanner.loadPath("GoBack", 0.5, 0.5);
    drivetrain.setOdometry(trajectory.getInitialPose());
  }

  public Command runPath(String pathName)
  {
    // boolean reversed = pathName.equals("FourBallPart1") || pathName.equals("FourBallPart2");
    double vel = 1;
    Trajectory trajectory = PathPlanner.loadPath(pathName, vel, 1, false);
    return drivetrain.getRamseteCommand(trajectory);
  }
}
