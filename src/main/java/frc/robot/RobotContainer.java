// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.Limelight;
import frc.robot.util.RollingAverage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Limelight limelight = new Limelight();
  private final Drivetrain drivetrain = new Drivetrain();
  private final Climber climber = new Climber();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter(limelight);
  private final Turret turret = new Turret(limelight);
  private Joystick operatorJoystick = new Joystick(1);  
  private final Storage storage = new Storage(operatorJoystick);

  private Joystick driverJoystick;
  
  private JoystickButton pivot;
  private JoystickButton intakeBalls;
  
  private JoystickButton expelBalls;

  private RollingAverage throttle = new RollingAverage(50);
  private RollingAverage wheel = new RollingAverage(15);

  private JoystickButton quickturn;
  private JoystickButton storageOverride; //Manual override for storage

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driverJoystick = new Joystick(0);
    intakeBalls = new JoystickButton(driverJoystick, 1);
    storageOverride = new JoystickButton(operatorJoystick, 10);
    // operatorJoystick = new Joystick(1);
    // Configure the button bindings
    configureButtonBindings();
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // the :: syntax allows us to pass in methods of a class as variables so that the command can continuously access input values
    //drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, this::getThrottle, this::getWheel, quickturn::get));
    //pivot.whenPressed(new Pivot(intake));
    //intakeBalls.whenPressed(new IntakeBalls(intake)).whenReleased(new StopIntaking(intake));
    //expelBalls.whenPressed(new ExpelBalls(storage));
    storage.setDefaultCommand(new ManualOverrideControl(storage, this::getStorageThrottle));
    storageOverride.whenPressed(new SwitchManual(storage));

    /*
      Shoot: 5(1 indexed)
      Intake: 6(1 indexed)
    */

  }

  /**
   * Gets the throttle input from the Driver Joystick throttle axis which is
   * used to move the robot forward and backwards
   * 
   * @return value from [-1, 1] that is used for input for the the robot forward or backwards movement
   */
  public double getThrottle() {
    throttle.updateValue(-driverJoystick.getRawAxis(5));
    return throttle.getAverage();
  }

  /**
   * Gets the wheel input from the Driver Joystick wheel axis which is used to turn the robot while
   * either driving or quickturning
   * 
   * @return value from [-1, 1] that is used for input for the robots turning movement
   */
  public double getWheel() {
    wheel.updateValue(driverJoystick.getRawAxis(0));
    return wheel.getAverage();
  }
  public double getStorageThrottle(){
    return 0.4* operatorJoystick.getRawAxis(5);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
