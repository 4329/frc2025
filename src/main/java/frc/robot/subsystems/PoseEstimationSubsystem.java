package frc.robot.subsystems;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.model.PoseEstimationLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.subsystems.lilih.LilihSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.LimelightHelpers.PoseEstimate;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class PoseEstimationSubsystem extends SubsystemBase implements LoggedSubsystem {

    PoseEstimationLogAutoLogged poseEstimationLogAutoLogged;

    private final Drivetrain drivetrain;
    private final LilihSubsystem lilihSubsystem;
    private SwerveDrivePoseEstimator estimator;
    private Field2d field = new Field2d();
    private Pose2d pathPlannerPose = new Pose2d();

    private AprilTagFieldLayout aprilTagFieldLayout;

    public PoseEstimationSubsystem(Drivetrain drivetrain, LilihSubsystem lilihSubsystem) {
        this.lilihSubsystem = lilihSubsystem;
        this.drivetrain = drivetrain;

        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        poseEstimationLogAutoLogged = new PoseEstimationLogAutoLogged();
        estimator =
                new SwerveDrivePoseEstimator(
                        Constants.DriveConstants.kDriveKinematics,
                        drivetrain.getGyro(),
                        drivetrain.getModulePositions(),
                        new Pose2d());

        PathPlannerLogging.setLogCurrentPoseCallback(
                (pose) -> {
                    field.setRobotPose(new Pose2d(0, 5, new Rotation2d()));
                    pathPlannerPose = pose != null ? pose : new Pose2d();
                });

        PathPlannerLogging.setLogTargetPoseCallback(
                (pose) -> {
                    field.getObject("target pose").setPose(pose);
                });

        PathPlannerLogging.setLogActivePathCallback(
                (poses) -> {
                    field.getObject("path").setPoses(poses);
                });
        Shuffleboard.getTab("field").add("field", field);
    }

    public void setInitialPose(Pose2d initialPose) {
        estimator =
                new SwerveDrivePoseEstimator(
                        Constants.DriveConstants.kDriveKinematics,
                        drivetrain.getGyro(),
                        drivetrain.getModulePositions(),
                        initialPose);
    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public Pose3d getTagPose(int id) {
        return aprilTagFieldLayout.getTagPose(id).orElse(null);
    }

    private void updateEstimation() {
        estimator.update(drivetrain.getGyro(), drivetrain.getModulePositions());

        if (lilihSubsystem.seeingAnything()) {
            PoseEstimate poseEstimate = lilihSubsystem.getRobotPose();
            if (poseEstimate.rawFiducials.length > 0 && poseEstimate.rawFiducials[0].ambiguity < .7)
                estimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
        }
    }

    @Override
    public void periodic() {
        updateEstimation();
    }

    @Override
    public LoggableInputs log() {
        poseEstimationLogAutoLogged.combined = getPose();
        poseEstimationLogAutoLogged.limOnly = lilihSubsystem.getRobotPose().pose;
        poseEstimationLogAutoLogged.driveOnly = drivetrain.getPose();
        poseEstimationLogAutoLogged.pathPlannerPosy = pathPlannerPose;
        Logger.recordOutput("zero", new Pose2d());
        Logger.recordOutput("zeroes", new Pose3d[] {});
        return poseEstimationLogAutoLogged;
    }
}
