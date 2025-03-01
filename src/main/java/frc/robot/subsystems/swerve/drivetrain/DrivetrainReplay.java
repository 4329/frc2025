package frc.robot.subsystems.swerve.drivetrain;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.model.DrivetrainLogAutoLogged;

public class DrivetrainReplay implements Drivetrain {

	public DrivetrainLogAutoLogged inputs = new DrivetrainLogAutoLogged();

    @Override
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {}

    @Override
    public void periodic() {}

    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates) {}

    @Override
    public void setModuleStates(ChassisSpeeds chassisSpeeds) {}

    @Override
    public void stop() {}

    @Override
    public void updateOdometry() {}

    @Override
    public void setInitialRotation(double initial) {}

    @Override
    public Rotation2d getGyro() {

        return inputs.gyro;
    }

    @Override
    public Rotation2d getRawGyro() {

        return inputs.gyro.minus(new Rotation2d(inputs.offset));
    }

    @Override
    public Pose2d getPose() {

        return inputs.pose;
    }

    @Override
    public void resetOdometry(Pose2d pose) {}

    @Override
    public void setPose(Pose2d pose) {}

    @Override
    public ChassisSpeeds getChassisSpeed() {

        return inputs.chassisSpeeds;
    }

    @Override
    public void brakeMode() {}

    @Override
    public void coastMode() {}

    @Override
    public void lock() {}

    @Override
    public void unlock() {}

    @Override
    public SwerveModulePosition[] getModulePositions() {

		return inputs.positions;
    }

    @Override
    public SwerveModuleState[] getModuleStates() {
		return inputs.states;
    }

	@Override
	public LoggableInputs log() {
		return inputs;
	}
}
