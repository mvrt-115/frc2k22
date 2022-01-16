package frc.robot.subsystems;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MathUtils;
import frc.robot.util.TalonFactory;

public class Drivetrain extends SubsystemBase {
    public static enum DrivetrainState {
        AUTON_PATH, JOYSTICK_DRIVE
    }

    private DrivetrainState state;

    private TalonFX rightMaster;
    private TalonFX leftMaster;
    private TalonFX rightFollower;
    private TalonFX leftFollower;

    private AHRS gyro;

    private Pose2d pose;
    private DifferentialDriveOdometry odometry;
    private Field2d field;

    public Drivetrain() {
        if (Constants.kIsPracticeBot) {
            rightMaster = TalonFactory.createTalonFX(Constants.Drivetrain.kPracRightMasterId, true);
            leftMaster = TalonFactory.createTalonFX(Constants.Drivetrain.kPracLeftMasterId, false);
            rightFollower = TalonFactory.createTalonFX(Constants.Drivetrain.kPracRightFollowerId, true);
            leftFollower = TalonFactory.createTalonFX(Constants.Drivetrain.kPracLeftFollowerId, false);
        } else {
            rightMaster = TalonFactory.createTalonFX(Constants.Drivetrain.kCompRightMasterId, true);
            leftMaster = TalonFactory.createTalonFX(Constants.Drivetrain.kCompLeftMasterId, false);
            rightFollower = TalonFactory.createTalonFX(Constants.Drivetrain.kCompRightFollowerId, true);
            leftFollower = TalonFactory.createTalonFX(Constants.Drivetrain.kCompLeftFollowerId, false);
        }

        rightFollower.follow(rightMaster);
        leftFollower.follow(leftMaster);

        pose = new Pose2d();
        gyro = new AHRS(SPI.Port.kMXP);
        odometry = new DifferentialDriveOdometry(getGyroAngle());
        field = new Field2d();
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
     * Returns the current state of the drivetrain
     * 
     * @return the current state
     */
    public DrivetrainState getState() {
        return state;
    }

    public void alignToBall() {
        int error = getErrorInPixels();
        if (Math.abs(error) > 0 && Math.abs(error) < Constants.Drivetrain.kAcceptableError) {
            double speedOfWheel = error / Constants.Drivetrain.kMaxPixelError;
            cheesyIshDrive(Constants.Drivetrain.kThrottle, speedOfWheel * error > 0 ? 1 : -1, true);
        }
    }

    /**
     * Method to be implemented by Sohan
     * 
     * @return Distance from where the robot is pointing to the ball in pixels
     */
    public int getErrorInPixels() {
        return 0;
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

        double left = 0, right = 0;

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
            left = motion.dx;
            right = motion.dx;
        } else {
            double delta_v = Constants.Drivetrain.kTrackWidthInches * motion.dtheta
                    / (2 * Constants.Drivetrain.kTrackScrubFactor);
            left = motion.dx + delta_v;
            right = motion.dx - delta_v;
        }

        double scaling_factor = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));

        setDrivetrainMotorSpeed(left / scaling_factor, right / scaling_factor);
    }

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
     * Stops the drivetrain
     */
    public void stopDrivetrainMotors() {
        setDrivetrainMotorSpeed(0, 0);
    }

    @Override
    public void periodic() {
        pose = odometry.update(getGyroAngle(), getDistanceTravelled(leftMaster, leftFollower),
                getDistanceTravelled(rightMaster, rightFollower));
        log();
    }

    /**
     * Logs data about the drivetrain subystem to SmartDashboard
     */
    public void log() {
        SmartDashboard.putNumber("Left Encoder:", leftMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right Encoder:", rightMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("Left Output", leftFollower.getMotorOutputPercent());
        SmartDashboard.putNumber("Right Output", rightFollower.getMotorOutputPercent());
        SmartDashboard.putString("Drivetrain State", state.toString());
        SmartDashboard.putNumber("Gyro Angle:", gyro.getAngle());
        SmartDashboard.putNumber("Left Distance Traveled", getDistanceTravelled(leftMaster, leftFollower));
        SmartDashboard.putNumber("Right Distance Traveled", getDistanceTravelled(rightMaster, rightFollower));
    }

    /** Getters and Setters */

    /** Resets the gyro */
    public void resetGyro() {
        gyro.reset();
    }

    /** Reset the odometry */
    public void resetOdometry() {
        odometry.update(new Rotation2d(), 0, 0);
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
                Constants.Drivetrain.kgearRatio,
                Constants.Drivetrain.kwheelCircumference);
    }
}
