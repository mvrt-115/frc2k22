package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drivetrain.DrivetrainState;
import frc.robot.subsystems.Intake.IntakeState;

/**
 * A command that runs throughout the teleoperated period, and constantly
 * checks the status of the joysticks to determine what output needs be
 * done by the motors, and sets the motors as such.
 */
public class JoystickDrive extends CommandBase {
  Drivetrain drivetrain;
  Supplier<Double> throttle;
  Supplier<Double> wheel;
  Supplier<Boolean> quickturn;
  Intake intake;
  /**
   * Makes sure that the Drivetrain.java subsystem object exists for use in the
   * command
   */
  public JoystickDrive(Drivetrain drivetrain, Intake intake, Supplier<Double> throttle, Supplier<Double> wheel, Supplier<Boolean> quickturn) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
    this.throttle = throttle;
    this.wheel = wheel;
    this.quickturn = quickturn;
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setState(DrivetrainState.JOYSTICK_DRIVE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Sends status of buttons and triggers on the trigger to the cheesyIshDrive
   * method in Drivetrain.java to have a certain drivetrain motor output.
   */
  @Override
  public void execute() 
  {
    double fwd = throttle.get();
    if(intake.getState() == IntakeState.INTAKING)
      fwd = 0.7*fwd;
    drivetrain.cheesyIshDrive(fwd, wheel.get(), quickturn.get());

    SmartDashboard.putNumber("Throttle", throttle.get());
    SmartDashboard.putNumber("Wheel", wheel.get());
    SmartDashboard.putBoolean("QuickTurn", quickturn.get());
  }

  // Called once the command ends or is interrupted.
  /**
   * When the program ends or is interrupted, the method stops the drivetrain motors
   * from moving.
   * @param interrupted     whether the command has been interrupted or not
   */
  @Override
  public void end(boolean interrupted) 
  {
    drivetrain.setDrivetrainMotorSpeed(0, 0);
  }

  // Returns true when the command should end (which is not until the robot command is interrupted)
  @Override
  public boolean isFinished() {
    return false;
  }
}