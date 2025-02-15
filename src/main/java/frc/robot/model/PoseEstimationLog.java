package frc.robot.model;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class PoseEstimationLog {
	public Rotation2d rotOffset;

    public Pose2d combined;
    public Pose2d limOnly;
    public Pose2d driveOnly;
    public Pose2d pathPlannerPosy;
}
