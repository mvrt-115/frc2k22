// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.simulation.AnalogInDataJNI;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FindTarget;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.ManualOverrideControl;
import frc.robot.commands.PIDTune;
import frc.robot.commands.SetHoodAngle;
import frc.robot.commands.SetRPM;
import frc.robot.commands.StopShooter;
import frc.robot.commands.SwitchManual;
import frc.robot.commands.TurretManual;
import frc.robot.commands.TurretSetupAlign;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Shooter.HoodState;
import frc.robot.subsystems.Shooter.ShooterState;
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
  // The robot's subsystems and commands are defined here...
  
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final Limelight limelight = new Limelight();
  private final Shooter shooter = new Shooter(limelight);
  private final Drivetrain dt = new Drivetrain();
  private final StopShooter stopShooter = new StopShooter(shooter);

  private final Turret turret = new Turret(limelight);

  private Joystick driverJoystick;
  private Joystick operatorJoystick;  

  private RollingAverage throttle = new RollingAverage(50);
  private RollingAverage wheel = new RollingAverage(15);

  private final Storage storage = new Storage(operatorJoystick);

  private JoystickButton storageOverride;

  private JoystickButton quickturn;

  public JoystickButton turretClockwise;
  public JoystickButton turretCounterclockwise;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    driverJoystick = new Joystick(0);
    operatorJoystick = new Joystick(1);

    storageOverride = new JoystickButton(driverJoystick, 0);

    turretClockwise = new JoystickButton(driverJoystick, 2);
    turretCounterclockwise = new JoystickButton(driverJoystick, 3);

    quickturn = new JoystickButton(driverJoystick, 9);

 

    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putData("Run Flywheel", new SetRPM(shooter));
    SmartDashboard.putData("Change Angle", new SetHoodAngle(shooter));
    SmartDashboard.putData("Config PIDF", new PIDTune(shooter.getMotor(), 
                                                      Constants.Flywheel.P, 
                                                      Constants.Flywheel.I,
                                                      Constants.Flywheel.D,
                                                      Constants.Flywheel.F,
                                                      "Flywheel",
                                                      stopShooter));
    dt.setDefaultCommand(new JoystickDrive(dt, this::getThrottle, this::getWheel, quickturn::get));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() 
  {
    //turret.setDefaultCommand(new FindTarget(turret));
    storage.setDefaultCommand(new ManualOverrideControl(storage, this::getStorageThrottle));
    storageOverride.whenPressed(new SwitchManual(storage));

    //turretClockwise.whenPressed(new TurretManual(turret, -0.5, turretClockwise::get));
    //turretCounterclockwise.whenPressed(new TurretManual(turret, 0.5, turretCounterclockwise::get));
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return new TurretSetupAlign(turret);

    return m_autoCommand;
  }

  /**
   * Use this to declare subsystem disabled behavior
   */
  public void disabledPeriodic() {
    shooter.setState(ShooterState.OFF);
    shooter.setHoodState(HoodState.OFF);
    shooter.log();
  }

  public double getStorageThrottle(){
    return 0.4* operatorJoystick.getRawAxis(5);
  }
}
