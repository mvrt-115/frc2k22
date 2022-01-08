package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Twist2d;
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
    }

    /**
     * Changes the state of the drivetrain
     * @param newState  the value of the new state
     */
    public void setState(DrivetrainState newState) {
        state = newState;
    }

    /**
     * Returns the current state of the drivetrain
     * @return  the current state
     */
    public DrivetrainState getState() {
        return state;
    }

    /**
     * Method for regular joystick driving during teleop through implementation of
     * the `syIshDrive, and then applies the speed to the motors
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
     * Sets a voltage percentage output to each motor given values for left and right motors
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
        log();
    }

    /**
     * Logs data about the drivetrain subystem to SmartDashboard
     */
    public void log() {
        SmartDashboard.putNumber("Left Output", leftFollower.getMotorOutputPercent());
        SmartDashboard.putNumber("right Output", rightFollower.getMotorOutputPercent());
        SmartDashboard.putString("Drivetrain State", state.toString());
    }
}
