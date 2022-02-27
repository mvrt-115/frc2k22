// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.Climber.Auton;
import frc.robot.commands.*;
import frc.robot.commands.pivot.PivotAuton;
import frc.robot.commands.pivot.PivotManual;
import frc.robot.commands.telescopic.TelescopicAuton;
import frc.robot.commands.telescopic.TelescopicManual;
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

  private Joystick driverJoystick;
  private Joystick operatorJoystick;  

  public static final boolean PIVOT_EXISTS = false;
  public static final boolean CLIMBER_TESTING = false && PIVOT_EXISTS;

  // climber operator manual buttons
  private JoystickButton forward;
  private JoystickButton extend;
  private JoystickButton retract;
  private JoystickButton backward;
  private JoystickButton startClimb;
  private JoystickButton stopClimb;

//Climber tests
  private JoystickButton manualButtonClimb;
  private int buttonCounter;
  private boolean manualLastState;


  private RollingAverage throttle = new RollingAverage(50);
  private RollingAverage wheel = new RollingAverage(15);

  private JoystickButton quickturn; 

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driverJoystick = new Joystick(1);
    operatorJoystick = new Joystick(0);

    if(PIVOT_EXISTS) {
      forward = new JoystickButton(operatorJoystick, 1);
      backward = new JoystickButton(operatorJoystick, 2); // change
    }
    extend =  new JoystickButton(operatorJoystick, 4);
    retract = new JoystickButton(operatorJoystick, 8);
    startClimb = new JoystickButton(operatorJoystick, 10);
    stopClimb = new JoystickButton(operatorJoystick, 9);

    if(CLIMBER_TESTING) {
      manualButtonClimb = new JoystickButton(operatorJoystick, 0);
      buttonCounter = 0;
    }

    quickturn = new JoystickButton(driverJoystick, 5); 
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
    
    
    /* if the telescopic button extend is pressed then it is checked to see if the top button is pressed for retracting the telescopic arm
    ** and based on that the correct command is called */


    retract.whenPressed(new TelescopicManual(climber, this::isRetractPressed, Constants.Climber.kTelescopicRetractManualSpeed))
      .whenReleased(new TelescopicManual(climber, this::isRetractPressed, 0));
    extend.whenPressed(new TelescopicManual(climber, this::isExtendPressed, Constants.Climber.kTelescopicExtendManualSpeed))
      .whenReleased(new TelescopicManual(climber, this::isExtendPressed, 0));
    /* if the pivot button forward is pressed then it is checked to see if the top button is pressed for pivoting backward for the pivot arm
    ** and based on that the correct command is called */
    if(PIVOT_EXISTS) {
        backward.whenPressed(new PivotManual(climber, this::isBackwardPressed, -Constants.Climber.kApproachRungSpeed))
          .whenReleased(new PivotManual(climber, this::isBackwardPressed, 0)); 
        forward.whenPressed(new PivotManual(climber, this::isForwardPressed, Constants.Climber.kApproachRungSpeed))
           .whenReleased(new PivotManual(climber, this::isForwardPressed, 0));
    }

    /** If the start climber button is pressed, then the start and stop climber parellel command is called and the instance of the stop climber 
     *  to help the command choose whether the stop climber or not
     */
    if(PIVOT_EXISTS && getStartClimb())
      startClimb.whenPressed(new StartStopClimb(this::getStopClimb, climber));

    /** the manual sequence method is called and checks the amount of times the button is pressed and runs the commands in that order and the 
     *  state of the button is stored as the past state and is called to check if the button was ever realeased
     */
    if(CLIMBER_TESTING)
    {
      if(manualButtonClimb.get())
        manualOneRungSeqeunceTest();
      //manualSequenceTest();
      //manualButtonClimb.whenPressed(new ClimberAuton(climber, climber.leftTelescopic, Constants.Climber.kTelescopicFullExtend, climber.leftTelescopicProximity));
      manualLastState = (manualButtonClimb.get());
    }
  }


  /////////////////////////////////////////////////GETTERS//////////////////////////////////////////////

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
   * Gets the state of the pivot button
   * @return all buttons states for buttons passed (boolean)
   */
  public boolean isForwardPressed() {
    return forward.get();
  }
  
  /**
   * Gets the state of the bakcward button
   * @return state of backward pivot button
   */
  public boolean isBackwardPressed() {
    return backward.get();
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
    boolean totalButtonState = true;
    for(int i = 0; i < buttons.length; i++)
      if(!buttons[i].get()) totalButtonState = false;
    return totalButtonState;
  }

  /**
   * Gets the state of the start button
   * @return boolean for state of start button
   */
  public boolean getStartClimb() {
    return startClimb.get();
  }

  /**
   * Gets the state of the stop button
   * @return boolean for state of stop button
   */
  public boolean getStopClimb() {
    return stopClimb.get();
  }

  /** Gets the state of the manual sequence button and checks to see if the last button state was released and 
     then the button counter is incremented to show that the button was pressed */
  public boolean getManualSequenceButton(){
    if(manualButtonClimb.get() != manualLastState && manualButtonClimb.get()) {
      buttonCounter++;
      return manualButtonClimb.get();
    }
    return false;
  }

  /** Each time the manual climb button is pressed, the next command in the sequence for the mid rung climb is called and 
   * the number of times the button is called is stored in the buttonCounter variable
   */
  
  public void manualOneRungSeqeunceTest() {
    if(getManualSequenceButton()) {
      switch(buttonCounter) {
        case 1:
          manualButtonClimb.whenPressed(new TelescopicAuton(climber, Constants.Climber.kTelescopicFullExtend, climber.leftTelescopicLimit));
          break;
        case 2:
          manualButtonClimb.whenPressed(new PivotAuton(climber, Auton.kPivotPivotingBack));
          break;
        case 3:
          manualButtonClimb.whenPressed(new TelescopicAuton(climber, Constants.Climber.Auton.kHookHighRungTele, climber.leftTelescopicProximity));
          break;
        case 4:
          manualButtonClimb.whenPressed(new TelescopicAuton(climber, Constants.Climber.kTelescopicFullRetract));
          break;
        case 5:
          manualButtonClimb.whenPressed(new PivotAuton(climber, Auton.kRotateToHighRungPivot, climber.pivotLimit));
          break;
        case 6:
          manualButtonClimb.whenPressed(new TelescopicAuton(climber, Auton.kExtendPivotHang, climber.pivotProximity));
          break;
        default:
          return;
      }
    }
  }
  
  /** Each time the manual climb button is pressed, the next command in the sequence for the traversal climb is called and 
   *  the number of times the button is called is stored in the buttonCounter variable
   */
  public void manualSequenceTest() {
    if(getManualSequenceButton()) {
      switch(buttonCounter) {
        case 1:
          manualButtonClimb.whenPressed(new TelescopicAuton(climber , Constants.Climber.Auton.kLiftOffRungTele));
          break;
        case 2:
          manualButtonClimb.whenPressed(new PivotAuton(climber, Constants.Climber.Auton.kPivotTeleBack));
          break;
        case 3:
          manualButtonClimb.whenPressed(new TelescopicAuton(climber, Constants.Climber.kTelescopicFullExtend));
          break;
        case 4:
          manualButtonClimb.whenPressed(new TelescopicAuton(climber, Constants.Climber.Auton.kRotateToHighRungTele, climber.leftTelescopicLimit));
          break;
        case 5:
          manualButtonClimb.whenPressed(new TelescopicAuton(climber, Constants.Climber.Auton.kHookHighRungTele, climber.leftTelescopicProximity));
          break;
        case 6:
          manualButtonClimb.whenPressed(new PivotAuton(climber, Constants.Climber.Auton.kShiftWeight));
          break;
        case 7:
          manualButtonClimb.whenPressed(new TelescopicAuton(climber, Constants.Climber.Auton.kRetractPivotLiftOff));
          return;
        case 8:
          manualButtonClimb.whenPressed(new PivotAuton(climber, Constants.Climber.Auton.kPivotPivotingBack));
          break;
        case 9:
          manualButtonClimb.whenPressed(new TelescopicAuton(climber, Constants.Climber.kTelescopicFullRetract));
          break;
        case 10: 
          manualButtonClimb.whenPressed(new PivotAuton(climber, Constants.Climber.Auton.kRotateToHighRungPivot, climber.pivotLimit));
          break;
        case 11:
          manualButtonClimb.whenPressed(new TelescopicAuton(climber, Constants.Climber.Auton.kExtendPivotHang, climber.pivotProximity));
          break;
        default: return;
      }
    }
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
