// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;

import com.fasterxml.jackson.databind.node.DoubleNode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  private Joystick driverJoystick; //Joysticks
  private Joystick operatorJoystick;  

  private final Limelight limelight = new Limelight();

  public final Drivetrain drivetrain = new Drivetrain();
  private final Shooter shooter = new Shooter(limelight);
  private final Turret turret = new Turret(limelight);
  private final Storage storage = new Storage(shooter);
  private final Intake intake = new Intake();

  private JoystickButton quickturn;
  // private JoystickButton alignDrivetrain;

  private JoystickButton intakeBalls;

  // private JoystickButton storageOverride;

  private JoystickButton shoot;
  // private JoystickButton expelBalls;
  // private final StopShooter stopShooter;

  private JoystickButton disableTurret;
  // public JoystickButton turretClockwise;
  // public JoystickButton turretCounterclockwise;

  public RollingAverage throttle, wheel;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    driverJoystick = new Joystick(0);
    operatorJoystick = new Joystick(1);

    // alignDrivetrain = new JoystickButton(operatorJoystick, 0);
    shoot = new JoystickButton(driverJoystick, 2);
    intakeBalls = new JoystickButton(driverJoystick, 3);
    quickturn = new JoystickButton(driverJoystick, 5);

    // expelBalls = new JoystickButton(operatorJoystick, 0);
    // disableTurret = new JoystickButton(operatorJoystick, 1);

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
    // storage.setDefaultCommand(new TrackBalls(storage, shooter, DriverStation.getAlliance().toString()));
    // turret.setDefaultCommand(new FindTarget(turret));

    // alignDrivetrain.whenPressed(new AlignIntakeToBall(drivetrain, true)).whenReleased(new AlignIntakeToBall(drivetrain, false));

    shoot.whenPressed(new SetRPM(shooter, storage, shoot)).whenReleased(new StopShooter(shooter, storage));
    // expelBalls.whenPressed(new ExpelBalls(storage));

    // intakeBalls.whenPressed(new IntakeBalls(intake)).whenReleased(new StopIntaking(intake));

    // disableTurret.whenPressed(new DisableTurret(turret));
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

  /**
   * Gets the ange of the right axis for the operator joystick
   * @return The angle from [-180, 180] where 0 degrees is the top and the right side represents
   * positive angles
   */
  public double getOperatorRightAxisAngle() {
    double x = operatorJoystick.getRawAxis(4);
    double y = -operatorJoystick.getRawAxis(5);

    double angle = Math.atan2(y, x);

    if(x < 0)
      angle += Math.PI;

    angle -= Math.PI / 2;
    angle *= -1;

    return angle * 180 / Math.PI;
  }

  /**
   * Gets the magnitude of the right axis for the operator joystick
   * @return The distance moved from the middle of the axis
   */
  public double getOperatorRightAxisMagnitude() {
    double x = operatorJoystick.getRawAxis(4);
    double y = operatorJoystick.getRawAxis(5);

    return Math.sqrt(x * x + y * y);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new FiveBallAuton(drivetrain, null, shooter, storage);
  }

  /**
   * Gets whether the intake button has been pressed
   * @return Value of the button
   */
  public boolean getintake() {
    return intakeBalls.get();
  }
  
  public Turret getTurret() {
    return turret;
  }

  /**
   * Use this to declare subsystem disabled behavior
   */
  public void disabledPeriodic() {
    shooter.setState(ShooterState.OFF);
    // shooter.setHoodState(HoodState.OFF);
  }
}
