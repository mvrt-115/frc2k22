// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.MathUtils;

public class DrivetrainSim {
    private WPI_TalonFX leftMaster, rightMaster;
    private WPI_Pigeon2 pigeon2;

    private TalonFXSimCollection leftMasterSim, rightMasterSim;
    private BasePigeonSimCollection pigeon2Sim;

    private Field2d field = new Field2d();
    private DifferentialDriveOdometry odometry;

    private DifferentialDrivetrainSim differentialDrivetrainSim = new DifferentialDrivetrainSim(
        DCMotor.getFalcon500(2),
        Constants.Drivetrain.kGearRatio,
        2.1, //Moment of Inertia kg m^2
        26.5, //MASS OF THE ROBOT IN KGS
        MathUtils.inchesToMeters(3), //6 inch diameter wheels
        0.2, // distance between wheels in meters
        VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) //noise stuff

        // The standard deviations for measurement noise ^:
		// x and y:          0.001 m
		// heading:          0.001 rad
		// l and r velocity: 0.1   m/s
		// l and r position: 0.005 m
    );

    public DrivetrainSim(WPI_TalonFX _lM, WPI_TalonFX _rM, WPI_Pigeon2 _pi)
    {   
        leftMaster = _lM;
        rightMaster = _rM;
        pigeon2 = _pi;

        leftMasterSim = leftMaster.getSimCollection();
        rightMasterSim = rightMaster.getSimCollection();
        pigeon2Sim = pigeon2.getSimCollection();

        odometry = new DifferentialDriveOdometry(pigeon2.getRotation2d());

        SmartDashboard.putData("Field", field);
    }

    public Field2d getField()
    {
        return field;
    }

    public void run()
    {
        differentialDrivetrainSim.setInputs(leftMasterSim.getMotorOutputLeadVoltage(), rightMasterSim.getMotorOutputLeadVoltage());
        differentialDrivetrainSim.update(0.02);

        leftMasterSim.setIntegratedSensorRawPosition(metersToTicks(differentialDrivetrainSim.getLeftPositionMeters()));
        leftMasterSim.setIntegratedSensorVelocity(velocityToTicks(differentialDrivetrainSim.getLeftVelocityMetersPerSecond()));
        rightMasterSim.setIntegratedSensorRawPosition(metersToTicks(differentialDrivetrainSim.getRightPositionMeters()));
        rightMasterSim.setIntegratedSensorVelocity(velocityToTicks(differentialDrivetrainSim.getRightVelocityMetersPerSecond()));
        pigeon2Sim.setRawHeading(differentialDrivetrainSim.getHeading().getDegrees());

        odometry.update(pigeon2.getRotation2d(), differentialDrivetrainSim.getLeftPositionMeters(), differentialDrivetrainSim.getRightPositionMeters());
        field.setRobotPose(odometry.getPoseMeters());
    }

    private int metersToTicks(double meters)
    {
        double wheelRotations = meters/(MathUtils.inchesToMeters(Constants.Drivetrain.kwheelCircumference));
        double motorRotations = wheelRotations * Constants.Drivetrain.kGearRatio; //perhaps should be 1, try
        int sensorCounts = (int)(motorRotations * Constants.Drivetrain.kTicksPerRevolution);
        return sensorCounts;
    }

    private int velocityToTicks(double velocityMetersPerSecond){
		double wheelRotationsPerSecond = velocityMetersPerSecond/(MathUtils.inchesToMeters(Constants.Drivetrain.kwheelCircumference));
		double motorRotationsPerSecond = wheelRotationsPerSecond * Constants.Drivetrain.kGearRatio; //perhaps should be 1, try
		double motorRotationsPer100ms = motorRotationsPerSecond / 10;
		int sensorCountsPer100ms = (int)(motorRotationsPer100ms * Constants.Drivetrain.kTicksPerRevolution);
		return sensorCountsPer100ms;
	}

    // private double ticksToDistanceMeters(double sensorCounts){
	// 	double motorRotations = (double)sensorCounts / Constants.Drivetrain.kTicksPerRevolution;
	// 	double wheelRotations = motorRotations / Constants.Drivetrain.kGearRatio; //should probably be 1 idk
	// 	double positionMeters = wheelRotations * (MathUtils.inchesToMeters(Constants.Drivetrain.kwheelCircumference));
	// 	return positionMeters;
	// }


}
