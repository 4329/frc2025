package frc.robot.Model;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

@AutoLog
public class DrivetrainLog {
    public Pose2d pose;
    public SwerveModuleState[] states;
}
