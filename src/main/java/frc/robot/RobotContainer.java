// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.Climber.Auton;
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
  
  private JoystickButton intakeBalls = new JoystickButton(operatorJoystick, 0);

  // climber operator manual buttons
  private JoystickButton pivotButton;
  private JoystickButton telescopicButton;
  private JoystickButton startClimb;
  private JoystickButton stopClimb;

//Climber tests
  private JoystickButton manualButtonClimb;
  private int buttonCounter;
  private boolean manualLastState;


  private RollingAverage throttle = new RollingAverage(50);
  private RollingAverage wheel = new RollingAverage(15);

  private JoystickButton quickturn = new JoystickButton(driverJoystick, 5);

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //driverJoystick = new Joystick(0);
    operatorJoystick = new Joystick(0);

    pivotButton = new JoystickButton(operatorJoystick, 3);
    telescopicButton =  new JoystickButton(operatorJoystick, 4);
    startClimb = new JoystickButton(operatorJoystick, 8);
    stopClimb = new JoystickButton(operatorJoystick, 7);


    manualButtonClimb = new JoystickButton(operatorJoystick, 0);
    buttonCounter = 0;
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
    /*drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, this::getThrottle, this::getWheel, quickturn::get));
    intakeBalls.whenPressed(new IntakeBalls(intake)).whenReleased(new StopIntaking(intake));
    */

    /* if the telescopic button extend is pressed then it is checked to see if the top button is pressed for retracting the telescopic arm
    ** and based on that the correct command is called */
    if(telescopicButton.get()) {
      if(getReverseManual()) 
        telescopicButton.whenPressed(new ClimberManual(climber, climber.leftTelescopic, this::getTelescopicReverseManual, Constants.Climber.kTelescopicRetractManualSpeed));
      else 
        telescopicButton.whenPressed(new ClimberManual(climber, climber.leftTelescopic, this::getTelescopicArmManual, Constants.Climber.kTelescopicExtendManualSpeed));
    }

    /* if the pivot button forward is pressed then it is checked to see if the top button is pressed for pivoting backward for the pivot arm
    ** and based on that the correct command is called */
    if(pivotButton.get()) {
      if(getReverseManual()) 
        pivotButton.whenPressed(new ClimberManual(climber, climber.pivot, this::getPivotReverseManual, -Constants.Climber.kApproachRungSpeed));
      else 
        pivotButton.whenPressed(new ClimberManual(climber, climber.pivot, this::getPivotArmManual, Constants.Climber.kApproachRungSpeed));
    }

    /** If the start climber button is pressed, then the start and stop climber parellel command is called and the instance of the stop climber 
     *  to help the command choose whether the stop climber or not
     */
    if(getStartClimb())
      startClimb.whenPressed(new StartStopClimb(this::getStopClimb, climber));

    //if(manualButtonClimb.get() && buttonCounter==1)
    //  manualOneRungSeqeunceTest();
    //manualSequenceTest();
    //manualButtonClimb.whenPressed(new ClimberAuton(climber, climber.leftTelescopic, Constants.Climber.kTelescopicFullExtend, climber.leftTelescopicProximity));

    /** the manual sequence method is called and checks the amount of times the button is pressed and runs the commands in that order and the 
     *  state of the button is stored as the past state and is called to check if the button was ever realeased
     */
    manualSequenceTest();
    manualLastState = manualButtonClimb.get();
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
  public boolean getPivotArmManual()
  {
    return pivotButton.get();
  }

  /**
   * Gets the state of the telescopic button
   * @return boolean for state of telescopic button
   */
  public boolean getTelescopicArmManual()
  {
    return telescopicButton.get();
  }

  /**
   * Gets the state of the reverse button
   * @return boolean for state of reverse button
   */
  public boolean getReverseManual() {
    return operatorJoystick.getRawAxis(3) >= Constants.Climber.kAxisThreshold;
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
   * Gets the collective state of the reverse trigger and the given button
   * @param button button that needs to be checked in conjunction with the trigger
   * @return boolean representing the collective state of the button and trigger
    */
  public boolean getReverseButton(JoystickButton button) {
    return button.get() && getReverseManual();
  }

  /** 
   * Gets the states of the telescopic and reverse buttons for running the telescopic in reverse
   * @return buttons' total states (boolean)
   */ 
  public boolean getTelescopicReverseManual() {
    return getReverseButton(telescopicButton);
  }

  /**
   * Gets the states of the pivot and reverse buttons for running the pivot arm in reverse
   * @return buttons' total state (boolean)
   */
  public boolean getPivotReverseManual() {
    return getReverseButton(pivotButton);
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

  /* Gets the state of the manual sequence button and checks to see if the last button state was released and 
     then the button counter is incremented to show that the button was pressed */
  public boolean getManualSequenceButton(){
    if(manualButtonClimb.get() != manualLastState && manualButtonClimb.get())
    {
      buttonCounter++;
      return manualButtonClimb.get();
    }
    return false;
  }

  /** Each time the manual climb button is pressed, the next command in the sequence for the mid rung climb is called and 
   * the number of times the button is called is stored in the buttonCounter variable
   */
  public void manualOneRungSeqeunceTest(){
    if(getManualSequenceButton() && buttonCounter == 1)
    {
      new ClimberAuton(climber, climber.leftTelescopic, Constants.Climber.kTelescopicFullExtend, climber.leftTelescopicLimit);
    }
    if(getManualSequenceButton() && buttonCounter == 2)
    {
      new ClimberAuton(climber, climber.pivot, Auton.kPivotPivotingBack);
    }
    if(getManualSequenceButton() && buttonCounter == 3)
    {
      new ClimberAuton(climber, climber.leftTelescopic, Constants.Climber.Auton.kHookHighRungTele, climber.leftTelescopicProximity);
    }
    if(getManualSequenceButton() && buttonCounter == 4)
    {
      new ClimberAuton(climber, climber.leftTelescopic, Constants.Climber.kTelescopicFullRetract);
    }
    if(getManualSequenceButton() && buttonCounter == 5)
    {
      new ClimberAuton(climber, climber.pivot, Auton.kRotateToHighRungPivot, climber.pivotLimit);
    }
    if(getManualSequenceButton() && buttonCounter == 6)
    {
      new ClimberAuton(climber, climber.leftTelescopic, Auton.kExtendPivotHang, climber.pivotProximity);
    }
  }

  /** Each time the manual climb button is pressed, the next command in the sequence for the traversal climb is called and 
   *  the number of times the button is called is stored in the buttonCounter variable
   */
  public void manualSequenceTest() {
    if(getManualSequenceButton() && buttonCounter == 1)
    {
      new ClimberAuton(climber, climber.leftTelescopic, Constants.Climber.Auton.kLiftOffRungTele);
    }
    else if(getManualSequenceButton() && buttonCounter == 2)
    {
      new ClimberAuton(climber, climber.pivot, Constants.Climber.Auton.kPivotTeleBack);
    }
    else if(getManualSequenceButton() && buttonCounter == 3)
    {
      new ClimberAuton(climber, climber.leftTelescopic, Constants.Climber.kTelescopicFullExtend);
    }
    else if(getManualSequenceButton() && buttonCounter == 4)
    {
      new ClimberAuton(climber, climber.leftTelescopic, Constants.Climber.Auton.kRotateToHighRungTele, climber.leftTelescopicLimit);
    }
    else if(getManualSequenceButton() && buttonCounter == 5)
    {
      new ClimberAuton(climber, climber.leftTelescopic, Constants.Climber.Auton.kHookHighRungTele, climber.leftTelescopicProximity);
    }
    else if(getManualSequenceButton() && buttonCounter == 6)
    {
      new ClimberAuton(climber, climber.pivot, Constants.Climber.Auton.kShiftWeight);
    }
    else if(getManualSequenceButton() && buttonCounter == 7)
    {
      new ClimberAuton(climber, climber.leftTelescopic, Constants.Climber.Auton.kRetractPivotLiftOff);
    }
    else if(getManualSequenceButton() && buttonCounter == 8)
    {
      new ClimberAuton(climber, climber.pivot, Constants.Climber.Auton.kPivotPivotingBack);
    }
    else if(getManualSequenceButton() && buttonCounter == 9)
    {
      new ClimberAuton(climber, climber.leftTelescopic, Constants.Climber.kTelescopicFullRetract);
    }
    else if(getManualSequenceButton() && buttonCounter == 10)
    {
      new ClimberAuton(climber, climber.pivot, Constants.Climber.Auton.kRotateToHighRungPivot, climber.pivotLimit);
    }
    else if(getManualSequenceButton() && buttonCounter == 11)
    {
      new ClimberAuton(climber, climber.leftTelescopic, Constants.Climber.Auton.kExtendPivotHang, climber.pivotProximity);
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
