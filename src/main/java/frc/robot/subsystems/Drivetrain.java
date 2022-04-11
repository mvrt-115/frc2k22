/**
 * Drivetrain.java
 * Literally what the name says - the code for the Drivetrain Subsystem
 */

package frc.robot.subsystems;

import java.sql.Driver;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MathUtils;

public class Drivetrain extends SubsystemBase {
    
    public static enum DrivetrainState {
        AUTON_PATH, JOYSTICK_DRIVE
    }

    private DrivetrainState state;

    private WPI_TalonFX rightMaster, leftMaster, rightFollower, leftFollower;

    private AHRS gyro;

    //Auton Stuff
    private Pose2d pose;
    private DifferentialDriveOdometry odometry;
    // private Field2d field;
    private SimpleMotorFeedforward feedforward;
    private DifferentialDriveKinematics kinematics;
    private PIDController leftController, rightController;
    private DifferentialDrive differentialDrive;

    public Drivetrain() {
        //     if (Constants.kIsPracticeBot) {
        //         rightMaster = TalonFactory.createTalonFX(Constants.Drivetrain.kPracRightMasterId, true);
        //         leftMaster = TalonFactory.createTalonFX(Constants.Drivetrain.kPracLeftMasterId, false);
        //         rightFollower = TalonFactory.createTalonFX(Constants.Drivetrain.kPracRightFollowerId, true);
        //         leftFollower = TalonFactory.createTalonFX(Constants.Drivetrain.kPracLeftFollowerId, false);
        //     } else {
        //         rightMaster = TalonFactory.createTalonFX(Constants.Drivetrain.kCompRightMasterId, true);
        //         leftMaster = TalonFactory.createTalonFX(Constants.Drivetrain.kCompLeftMasterId, false);
        //         rightFollower = TalonFactory.createTalonFX(Constants.Drivetrain.kCompRightFollowerId, true);
        //         leftFollower = TalonFactory.createTalonFX(Constants.Drivetrain.kCompLeftFollowerId, false);
        //     }

        rightMaster = new WPI_TalonFX(Constants.Drivetrain.kPracRightMasterId);
        rightFollower = new WPI_TalonFX(Constants.Drivetrain.kPracRightFollowerId);
        leftMaster = new WPI_TalonFX(Constants.Drivetrain.kPracLeftMasterId);
        leftFollower = new WPI_TalonFX(Constants.Drivetrain.kPracLeftFollowerId);
        
        rightMaster.configFactoryDefault();
        rightFollower.configFactoryDefault();
        leftMaster.configFactoryDefault();
        leftFollower.configFactoryDefault();

        rightMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx, Constants.kTimeoutMs);
        rightFollower.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx, Constants.kTimeoutMs);
        leftMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx, Constants.kTimeoutMs);
        leftFollower.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx, Constants.kTimeoutMs);

        rightMaster.setInverted(TalonFXInvertType.Clockwise); 
        rightFollower.setInverted(TalonFXInvertType.Clockwise);
        leftMaster.setInverted(TalonFXInvertType.CounterClockwise);
        leftFollower.setInverted(TalonFXInvertType. CounterClockwise);

        rightMaster.configSupplyCurrentLimit(Constants.kCurrentLimit, Constants.kTimeoutMs);
        rightFollower.configSupplyCurrentLimit(Constants.kCurrentLimit, Constants.kTimeoutMs);
        leftMaster.configSupplyCurrentLimit(Constants.kCurrentLimit, Constants.kTimeoutMs);
        leftFollower.configSupplyCurrentLimit(Constants.kCurrentLimit, Constants.kTimeoutMs);

        rightFollower.follow(rightMaster);
        leftFollower.follow(leftMaster);

        //hmmmmmm
        rightMaster.setSafetyEnabled(false);
        rightFollower.setSafetyEnabled(false);
        leftMaster.setSafetyEnabled(false);
        leftFollower.setSafetyEnabled(false);
        differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

        pose = new Pose2d();
        gyro = new AHRS(SPI.Port.kMXP);
        odometry = new DifferentialDriveOdometry(getGyroAngle());
        // field = new Field2d();

        feedforward = new SimpleMotorFeedforward(Constants.Drivetrain.kS, Constants.Drivetrain.kV, Constants.Drivetrain.kA);
        kinematics = new DifferentialDriveKinematics(MathUtils.inchesToMeters(Constants.Drivetrain.kTrackWidthInches)); //because kTrackWidthInches is in, well, inches (duh) lol
       
        leftController = new PIDController(Constants.Drivetrain.kP, Constants.Drivetrain.kI, Constants.Drivetrain.kD);
        rightController = new PIDController(Constants.Drivetrain.kP, Constants.Drivetrain.kI, Constants.Drivetrain.kD);
    
        leftMaster.setSelectedSensorPosition(0);
        leftFollower.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);
        rightFollower.setSelectedSensorPosition(0);

        // SmartDashboard.putData("field", field);
        
        gyro.reset();
    }

    /**
     * Method for regular joystick driving during teleop through implementation of
     * the cheesyIshDrive, and then applies the speed to the motors
     * 
     * @param throttle  (double) power towards forward and backward movement
     * @param wheel     (double) power towards turning movement
     * @param quickTurn (boolean) status of quickTurnButton
     */
    public void cheesyIshDrive(double throttle, double wheel, boolean quickTurn) {
        throttle = MathUtils.handleDeadband(throttle, Constants.Drivetrain.kThrottleDeadband);
        wheel = MathUtils.handleDeadband(wheel, Constants.Drivetrain.kWheelDeadband);

        if(quickTurn)
            wheel *= 0.8;

        double left = 0, right = 0;
        // setDrivetrainMotorSpeed((throttle+wheel) / 2, (throttle-wheel) / 2);
        final double denominator = Math.sin(Math.PI / 2.0 * Constants.Drivetrain.kWheelNonlinearity);
        // Apply a sin function that's scaled to make it feel better.
        if (!quickTurn) {
            wheel = Math.sin(Math.PI / 2.0 * Constants.Drivetrain.kWheelNonlinearity * wheel);
            wheel = Math.sin(Math.PI / 2.0 * Constants.Drivetrain.kWheelNonlinearity * wheel);
            wheel = wheel / (denominator * denominator) * Math.abs(throttle);
        }

        wheel *= Constants.Drivetrain.kWheelGain;
        Twist2d motion = new Twist2d(throttle, 0, wheel);
        if (Math.abs(motion.dtheta) < 1E-9) {
            left = motion.dx ;
            right = motion.dx;
        } else {
            double delta_v = Constants.Drivetrain.kTrackWidthInches * motion.dtheta
                    / (2 * Constants.Drivetrain.kTrackScrubFactor);
            left = motion.dx + delta_v;
            right = motion.dx - delta_v;
        }

        double scaling_factor = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));
        // DO NOT DELETE WE DON'T KNOW WHY BUT THIS MAKES IT WORK3
        // SmartDashboard.putNumber("left", left);
        // SmartDashboard.putNumber("right", right);
        setDrivetrainMotorSpeed(left / scaling_factor, right / scaling_factor);
    }

    @Override
    public void periodic() {
        //hmmmmmmmmmmmm (imo it doesn't really make a difference, idk)
        differentialDrive.feed();
        leftMaster.feed();
        rightMaster.feed();
        leftFollower.feed();
        rightFollower.feed();
        if (DriverStation.isAutonomous()) {
            
            pose = odometry.update(getGyroAngle(), getDistanceTravelled(leftMaster, leftFollower),
                    getDistanceTravelled(rightMaster, rightFollower));
        }
        // field.setRobotPose(odometry.getPoseMeters());
        log();
    }

    /**
     * Logs data about the drivetrain subystem to SmartDashboard
     */
    public void log() {
        // SmartDashboard.putNumber("Left Encoder:", leftMaster.getSelectedSensorPosition());
        // SmartDashboard.putNumber("Right Encoder:", rightMaster.getSelectedSensorPosition());
        // SmartDashboard.putNumber("Left Output", leftFollower.getMotorOutputPercent());
        // SmartDashboard.putNumber("Right Output", rightFollower.getMotorOutputPercent());
        // SmartDashboard.putNumber("Gyro Angle:", -gyro.getAngle());
        // SmartDashboard.putNumber("Pose Gyro Angle", pose.getRotation().getDegrees());
        // SmartDashboard.putNumber("Left Distance Traveled", getDistanceTravelled(leftMaster, leftFollower));
        // SmartDashboard.putNumber("Right Distance Traveled", getDistanceTravelled(rightMaster, rightFollower));
        // SmartDashboard.putNumber("X value", pose.getX());
        // SmartDashboard.putNumber("Y value", pose.getY());
        // SmartDashboard.putData("SD Field", field);
    }

    /* GETTERS AND SETTERS (AND RESETTERS) */

    /**
     * Sets a voltage percentage output to each motor given values for left and
     * right motors
     * 
     * @param left  the percentage [-1, 1] for the left motors
     * @param right the percentage [-1, 1] from the right motors
     */
    public void setDrivetrainMotorSpeed(double left, double right) {
        leftMaster.set(ControlMode.PercentOutput, left);
        leftFollower.set(ControlMode.PercentOutput, left);
        rightFollower.set(ControlMode.PercentOutput, right);
        rightMaster.set(ControlMode.PercentOutput, right);
    }
    
    /**
     * Used in auton
     * 
     * @param leftMetersPerSecond - speed of the left side
     * @param rightMetersPerSecond - speed of right side
     */
    public void setDrivetrainVelocity(double leftMetersPerSecond, double rightMetersPerSecond){
        double leftRadiansPerSec = MathUtils.metersToRadians(leftMetersPerSecond, Constants.Drivetrain.kwheelCircumference);
        double rightRadiansPerSec = MathUtils.metersToRadians(rightMetersPerSecond, Constants.Drivetrain.kwheelCircumference);

        double leftVolts = feedforward.calculate(leftRadiansPerSec);
        double rightVolts = feedforward.calculate(rightRadiansPerSec);
        
        leftMaster.set(ControlMode.PercentOutput, leftVolts / 12.0);
        leftFollower.set(ControlMode.PercentOutput, leftVolts / 12.0);
        rightMaster.set(ControlMode.PercentOutput, rightVolts / 12.0);
        rightFollower.set(ControlMode.PercentOutput, rightVolts / 12.0);
    }

    /**
     * Sets the motor output as a percentage of the supplied voltage.
     * @param leftVoltage   voltage to the left motors
     * @param rightVoltage  voltage to the right motors
     */
    public void setOutputVoltage(double leftVoltage, double rightVoltage) {
        setDrivetrainMotorSpeed(leftVoltage/Constants.kVoltageComp, rightVoltage/Constants.kVoltageComp);
    }

    /**
     * Stops the drivetrain.
     */
    public void stopDrivetrain() {
        setDrivetrainMotorSpeed(0, 0);
    }

    /**
     * Changes the state of the drivetrain
     * 
     * @param newState the value of the new state
     */
    public void setState(DrivetrainState newState) {
        state = newState;
    }

    /**
     * The holy mother of all the Auton stuff
     * @param trajectory the trajectory to follow
     * @return RamseteCommand, the ramsete command to run the auton stuff
     */
    public Command getRamseteCommand(Trajectory trajectory) {
        RamseteCommand command = new RamseteCommand(
            trajectory, 
            this::getPose, 
            new RamseteController(2.0, 7.0), //values used from 2k21 rewrite
            getFeedForward(), 
            getKinematics(), 
            this::getSpeeds, 
            getLeftPIDController(), 
            getRightPIDController(), 
            this::setOutputVoltage, 
            this
        );

        return command;
    }

    /**
     * Returns the rotation as a Rotation2d object
     * 
     * @return Rotation2d of current rotation
     */
    public Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    /**
     * Returns the distance traveled by the two motor parameters.
     * Precondition: motors are on the same side
     * 
     * @param m1 motor
     * @param m2 other motor
     * @return distance traveled in meters
     */
    public double getDistanceTravelled(TalonFX m1, TalonFX m2) {
        double ticks = (m1.getSelectedSensorPosition() + m2.getSelectedSensorPosition()) / 2.0;

        // returns the converted values
        return MathUtils.convertTicksToMeters(
                ticks,
                Constants.Drivetrain.kTicksPerRevolution,
                Constants.Drivetrain.kGearRatio,
                Constants.Drivetrain.kwheelCircumference
        );
    }

    /**
     * Returns the current state of the drivetrain
     * 
     * @return the current state
     */
    public DrivetrainState getState() {
        return state;
    }

    /** 
     * Returns the Right PID Controller
     */
    public PIDController getRightPIDController() {
        return rightController;
    }

    /** 
     * Returns the Left PID Controller
     */
    public PIDController getLeftPIDController() {
        return leftController;
    }

    /**
     * Returns the differential drive's speeds (for the ramsete command)
     * @return DifferentialDriveWheelSpeeds for the left and right wheels
     */
    public DifferentialDriveWheelSpeeds getSpeeds() {
        double leftSpeed = MathUtils.RPMtoMetersPerSecond(
            leftMaster.getSelectedSensorVelocity(), 
            Constants.Drivetrain.kTicksPerRevolution, 
            Constants.Drivetrain.kGearRatio,
            Constants.Drivetrain.kwheelCircumference);
        
        double rightSpeed = MathUtils.RPMtoMetersPerSecond(
            leftMaster.getSelectedSensorVelocity(), 
            Constants.Drivetrain.kTicksPerRevolution, 
            Constants.Drivetrain.kGearRatio,
            Constants.Drivetrain.kwheelCircumference);

        return new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);
    }

    /** 
     * Returns the DifferentialDriveKinematics
     * @return kinematics
     */
    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    /**
     * Returns the feed forward
     * @return feedforward
     */
    public SimpleMotorFeedforward getFeedForward() {
        return feedforward;
    }

    /**
     * Returns the current pose of the robot
     * @return pose, the Robot pose
     */
    public Pose2d getPose() {
        return pose;
    }
    
    // /**
    //  * Returns the field
    //  * @return the robot field, shown in smartdashboard.
    //  */
    // public Field2d getField() {
    //     return field;
    // }
    
    /** 
     * Resets the gyro 
     */
    public void resetGyro() {
        gyro.reset();
    }

    /** 
     * Reset the odometry and the encoders
     */
    public void resetOdometry() {
        resetEncoders();
        odometry.update(new Rotation2d(), 0, 0);
    }

    /** 
     * Reset the odometry and the encoders
     */
    public void setOdometry(Pose2d newPose) {
        resetEncoders();
        odometry.resetPosition(newPose, new Rotation2d(0));
        // odometry.resetPosition(newPose, newPose.getRotation());
    }

    /** 
     * Resets the encoders 
     */
    public void resetEncoders() {
        leftMaster.setSelectedSensorPosition(0);
        leftFollower.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);
        rightFollower.setSelectedSensorPosition(0);
    }

    public double getLinSpeed() // Positive speed in direction of zeroed turret, negative speed in direction of intake
    {
        return -1*(getSpeeds().leftMetersPerSecond+getSpeeds().rightMetersPerSecond)/2;
    }
}