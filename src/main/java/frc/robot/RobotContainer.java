// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.telescopic.*;
import frc.robot.subsystems.*;
import frc.robot.util.Limelight;
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
  
  private JoystickButton  pivot; //buttons
  private JoystickButton alignDrivetrain;

  public final Drivetrain drivetrain = new Drivetrain();
  private final Limelight limelight = new Limelight();
  private final Shooter shooter = new Shooter(limelight);
  private final Climber climber = new Climber();
  private final Turret turret = new Turret(limelight);

  
  // climber operator manual buttons
  private JoystickButton extend;
  private JoystickButton retract;

  private JoystickButton upManualStorage;
  private JoystickButton downManualStorage;

  public RollingAverage throttle, wheel;

  private final Storage storage = new Storage();

  private final Intake intake = new Intake();

  private JoystickButton quickturn;
  private JoystickButton shoot;

  public JoystickButton disableTurret;
  public JoystickButton zeroTurret;

  private FindTarget findTarget;
  private SetRPM setRPM;
  private StopShooter stopShooter;
  private Pivot pivotDown;
  private PivotUp pivotUp;
  private ManualStorage manualStorageUp;
  private ManualStorage manualStorageDown;
  private UnratchetExtend telescopeUp;
  private RatchetRetract telescopeDown;
  private TelescopicManual stopClimber;
  private Command twoBallAuto;
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    driverJoystick = new Joystick(0);
    operatorJoystick = new Joystick(1); //MAKE SURE IT IS ON D MODE (so that the right trigger acts as a button)

    shoot = new JoystickButton(driverJoystick, 8);
    pivot = new JoystickButton(driverJoystick, 6);
    alignDrivetrain = new JoystickButton(driverJoystick, 6);

    quickturn = new JoystickButton(driverJoystick, 5);
    throttle = new RollingAverage(50);
    wheel = new RollingAverage(15);

    extend =  new JoystickButton(operatorJoystick, 9);
    retract = new JoystickButton(operatorJoystick, 10);

    disableTurret = new JoystickButton(operatorJoystick, 2);
    zeroTurret = new JoystickButton(operatorJoystick, 3);
    upManualStorage = new JoystickButton(operatorJoystick, 7);
    downManualStorage = new JoystickButton(operatorJoystick, 8);

    findTarget = new FindTarget(turret);
    setRPM = new SetRPM(shooter, storage, turret, shoot);
    stopShooter = new StopShooter(shooter, storage);
    pivotDown = new Pivot(intake, storage);
    pivotUp = new PivotUp(intake, storage);
    manualStorageUp = new ManualStorage(storage, true, upManualStorage::get);
    manualStorageDown = new ManualStorage(storage, false, downManualStorage::get);
    telescopeDown = new RatchetRetract(climber, this::isRetractPressed, Constants.Climber.kTelescopicRetractManualSpeed);
    telescopeUp = new UnratchetExtend(climber, this::isExtendPressed, Constants.Climber.kTelescopicExtendManualSpeed);
    stopClimber = new TelescopicManual(climber, this::isRetractPressed, 0);

    twoBallAuto = new ParallelCommandGroup(
      new SequentialCommandGroup(
        new RunDrive(drivetrain, 1.5).withTimeout(1.5),
        new SetRPM(shooter, storage, turret)
      ),
      new Pivot(intake, storage)
    );
    
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
    // storage.setDefaultCommand(new TrackBalls(storage, shooter, alliance));
    // the :: syntax allows us to pass in methods of a class as variables so that the command can continuously access input values
    // alignDrivetrain.whenPressed(new AlignIntakeToBall(drivetrain, true)).whenReleased(new JoystickDrive(drivetrain, this::getThrottle, this::getWheel, quickturn::get));
    drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, intake, 
    this::getThrottle, this::getWheel, quickturn::get));

    storage.setDefaultCommand(new TrackBalls(storage, shooter));
    turret.setDefaultCommand(new FindTarget(turret));
    
    shoot.whenPressed(setRPM).whenReleased(stopShooter);

    // SmartDashboard.putData("Testing Shooter", new SetRPM(shooter, storage, true));
    pivot.whenPressed(pivotDown).whenReleased(pivotUp);
      
    upManualStorage.whenPressed(manualStorageUp);  // true for up
    downManualStorage.whenPressed(manualStorageDown); // false for down
    
    // alignDrivetrain.whenPressed(new AlignIntakeToBall(drivetrain, true)).whenReleased(new JoystickDrive(drivetrain, this::getThrottle, this::getWheel, quickturn::get));
    /* when the retract and extend buttons are pressed then the telescopic manual command is called accordingly with 
       the given value */

    // retract.whenPressed(new TelescopicManual(climber, this::isRetractPressed, Constants.Climber.kTelescopicRetractManualSpeed))
    retract.whenPressed(telescopeDown)
      .whenReleased(stopClimber);

    //extend.whenPressed(new TelescopicManual(climber, this::isExtendPressed, Constants.Climber.kTelescopicExtendManualSpeed))
    extend.whenPressed(telescopeUp)
    .whenReleased(stopClimber); 

    disableTurret.whenPressed(new DisableTurret(turret)).whenReleased(new FindTarget(turret));
    zeroTurret.whenPressed(new ZeroTurret(turret)).whenReleased(new FindTarget(turret));
  }
  
  
  /////////////////////////////////////////////////GETTERS//////////////////////////////////////////////

  /**
   * Gets the throttle input from the Driver Joystick throttle axis which is
   * used to move the robot forward and backwards
   * 
   * @return value from [-1, 1] that is used for input for the the robot forward or backwards movement
   */
  public double getThrottle() {
    throttle.updateValue(-driverJoystick.getRawAxis(3) * 1);
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
   * Gets the state of the telescopic button
   * @return boolean for state of telescopic button 
   */
  public boolean isExtendPressed() {
    return extend.get();
  }

  public boolean isRetractPressed() {
    return retract.get();
  }

  /** 
   * Gets the state of all buttons (if all buttoons are pressed, then is true, otherwise false)
   * @param buttons the array of all buttons that need to be checked
   * @return all buttons states for buttons passed (boolean)
   */
  public boolean getAllButtonStates(JoystickButton[] buttons) {
    for(int i = 0; i < buttons.length; i++)
      if(!buttons[i].get()) return false;

    return true;
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return twoBallAuto;

  }

  public double getStorageThrottle(){
    return 0.4* operatorJoystick.getRawAxis(5);
  }
}
