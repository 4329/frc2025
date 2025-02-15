package frc.robot.model;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class DrivetrainLog {
    public SwerveModuleState[] states;
    public double offset;
    public Rotation2d rot;
}
