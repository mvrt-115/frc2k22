// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.commands.SetRPM;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.subsystems.Shooter.HoodState;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.util.Limelight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.RollingAverage;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
 // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private Joystick driverJoystick; //Joysticks
  private Joystick operatorJoystick;  
  
  private JoystickButton intakeBalls; //buttons
  // private JoystickButton alignDrivetrain;
  // private JoystickButton expelBalls;

  private Drivetrain drivetrain;


  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final Limelight limelight = new Limelight();
  private final Shooter shooter = new Shooter(limelight);
  private final Turret turret = new Turret(limelight);
  // private final StopShooter stopShooter = new StopShooter(shooter);

  // private final Turret turret = new Turret(limelight);

  public RollingAverage throttle, wheel;

  private final Storage storage = new Storage(shooter);

  private final Intake intake = new Intake();

  // private JoystickButton storageOverride;

  private JoystickButton quickturn;
  private JoystickButton shoot;

  private JoystickButton disableTurret;
  // public JoystickButton turretClockwise;
  // public JoystickButton turretCounterclockwise;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    driverJoystick = new Joystick(0);
    operatorJoystick = new Joystick(1);
    shoot = new JoystickButton(driverJoystick, 2);

    // disableTurret = new JoystickButton(operatorJoystick, 1);
    // // turretClockwise = new JoystickButton(driverJoystick, 2);
    // turretCounterclockwise = new JoystickButton(driverJoystick, 3);

 

    // Configure the button bindings
    
    driverJoystick = new Joystick(0);

    intakeBalls = new JoystickButton(driverJoystick, 6);
    // alignDrivetrain = new JoystickButton(operatorJoystick, 0);
    // expelBalls = new JoystickButton(operatorJoystick, 0);

    quickturn = new JoystickButton(driverJoystick, 1);
    drivetrain = new Drivetrain();
    throttle = new RollingAverage(50);
    wheel = new RollingAverage(15);

    
    configureButtonBindings();

    // SmartDashboard.putData("Run Flywheel", new SetRPM(shooter));
    // SmartDashboard.putData("Change Angle", new SetHoodAngle(shooter));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // the :: syntax allows us to pass in methods of a class as variables so that the command can continuously access input values
    drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, this::getThrottle, this::getWheel, quickturn::get));

   // shoot.whenPressed(new SetRPM(shooter, storage, shoot)).whenReleased(new StopShooter(shooter, storage));
    intakeBalls.whenPressed(new IntakeBalls(intake)).whenReleased(new StopIntaking(intake));
    // expelBalls.whenPressed(new ExpelBalls(storage));
    // alignDrivetrain.whenPressed(new AlignIntakeToBall(drivetrain, true)).whenReleased(new AlignIntakeToBall(drivetrain, false));
    /*
      Shoot: 4
      Intake: 5
      Expell Balls: <find>
      Climb: <find for operator>
      Turret Manual: ????
      Align To ball: 0
    */

    //turretClockwise.whenPressed(new TurretManual(turret, -0.5, turretClockwise::get));
    //turretCounterclockwise.whenPressed(new TurretManual(turret, 0.5, turretCounterclockwise::get));

    // turret.setDefaultCommand(new FindTarget(turret));
    
    
    // disableTurret.whenPressed(new DisableTurret(turret));
  }

  /**
   * Gets the throttle input from the Driver Joystick throttle axis which is
   * used to move the robot forward and backwards
   * 
   * @return value from [-1, 1] that is used for input for the the robot forward or backwards movement
   */
  public double getThrottle() {
    throttle.updateValue(-driverJoystick.getRawAxis(5) * .7);
    return throttle.getAverage();
  }
  
  /**
   * Gets the wheel input from the Driver Joystick wheel axis which is used to turn the robot while
   * either driving or quickturning
   * 
   * @return value from [-1, 1] that is used for input for the robots turning movement
   */
  public double getWheel() {
    wheel.updateValue(driverJoystick.getRawAxis(0) * .8);
    return wheel.getAverage();

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   // Trajectory tr = PathPlanner.loadPath("TryPath", 4, 4);
    return null;//new TurretSetupAlign(turret);//new AutonPath6(drivetrain);
    //return new DriveTrajectory(drivetrain);
  }
  
  /**
   * Use this to declare subsystem disabled behavior
   */
  public void disabledPeriodic() {
    shooter.setState(ShooterState.OFF);
    // shooter.setHoodState(HoodState.OFF);
  }

  public double getStorageThrottle(){
    return 0.4* operatorJoystick.getRawAxis(5);
  }
}
