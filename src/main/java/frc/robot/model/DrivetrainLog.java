package frc.robot.model;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class DrivetrainLog {
	public Pose2d pose;
    public SwerveModuleState[] states;
	public SwerveModulePosition[] positions;
	public Rotation2d gyro;
    public double offset;
	public ChassisSpeeds chassisSpeeds;
}
