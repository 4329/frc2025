package frc.robot.subsystems.swerve.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.model.DrivetrainLogAutoLogged;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class DrivetrainReplay implements Drivetrain {
    DrivetrainLogAutoLogged drivetrainLogAutoLogged = new DrivetrainLogAutoLogged();

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

        return new Rotation2d();
    }

    @Override
    public Rotation2d getRawGyro() {

        return new Rotation2d();
    }

    @Override
    public Pose2d getPose() {

        return new Pose2d();
    }

    @Override
    public void resetOdometry(Pose2d pose) {}

    @Override
    public void setPose(Pose2d pose) {}

    @Override
    public ChassisSpeeds getChassisSpeed() {

        return new ChassisSpeeds();
    }

    @Override
    public double getFrontLeftAngle() {

        return 0;
    }

    @Override
    public double getFrontRightAngle() {

        return 0;
    }

    @Override
    public double getBackLeftAngle() {

        return 0;
    }

    @Override
    public double getBackRightAngle() {

        return 0;
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

        return new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };
    }

    @Override
    public SwerveModuleState[] getModuleStates() {

        return new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        };
    }

    @Override
    public void resetKeepAngle() {}

    @Override
    public LoggableInputs log() {
        return drivetrainLogAutoLogged;
    }
}
