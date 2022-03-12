// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//import frc.robot.commands.SetRPM;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
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
  private JoystickButton alignDrivetrain;

  public final Drivetrain drivetrain = new Drivetrain();
  private final Limelight limelight = new Limelight();
  private final Shooter shooter = new Shooter(limelight);
  private final Turret turret = new Turret(limelight);


  public RollingAverage throttle, wheel;

  private final Storage storage = new Storage();

  private final Intake intake = new Intake();

  private JoystickButton quickturn;
  private JoystickButton shoot;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    driverJoystick = new Joystick(0);
    operatorJoystick = new Joystick(1);

    shoot = new JoystickButton(driverJoystick, 2);
    intakeBalls = new JoystickButton(driverJoystick, 3);
    alignDrivetrain = new JoystickButton(driverJoystick, 6);

    quickturn = new JoystickButton(driverJoystick, 5);
    throttle = new RollingAverage(50);
    wheel = new RollingAverage(15);

    
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // storage.setDefaultCommand(new TrackBalls(storage, shooter, alliance));
    // the :: syntax allows us to pass in methods of a class as variables so that the command can continuously access input values
    drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, this::getThrottle, this::getWheel, quickturn::get));

    storage.setDefaultCommand(new TrackBalls(storage, shooter));
    turret.setDefaultCommand(new FindTarget(turret));
    
    shoot.whenPressed(new SetRPM(shooter, storage, shoot)).whenReleased(new StopShooter(shooter, storage));
    intakeBalls.whenPressed(new IntakeBalls(intake, storage)).whenReleased(new StopIntaking(intake, storage));
    alignDrivetrain.whenPressed(new AlignIntakeToBall(drivetrain, true)).whenReleased(new JoystickDrive(drivetrain, this::getThrottle, this::getWheel, quickturn::get));
  }

  /**
   * Gets the throttle input from the Driver Joystick throttle axis which is
   * used to move the robot forward and backwards
   * 
   * @return value from [-1, 1] that is used for input for the the robot forward or backwards movement
   */
  public double getThrottle() {
    throttle.updateValue(-driverJoystick.getRawAxis(5) * 1);
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

  public boolean getintake(){
    return intakeBalls.get();
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new FiveBallAuton(drivetrain, intake, shooter, storage, turret);

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
