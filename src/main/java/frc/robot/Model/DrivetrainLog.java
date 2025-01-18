package frc.robot.Model;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class DrivetrainLog {
    public Pose2d pose;
    public SwerveModuleState[] states;
}
