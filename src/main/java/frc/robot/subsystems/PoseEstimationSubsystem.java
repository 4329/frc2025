package frc.robot.subsystems;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.model.PoseEstimationLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.subsystems.lilih.LilihSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.LimelightHelpers.PoseEstimate;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class PoseEstimationSubsystem extends SubsystemBase implements LoggedSubsystem {

    public static final double OFFSET = 0.1651;

    PoseEstimationLogAutoLogged poseEstimationLogAutoLogged;

    private final Drivetrain drivetrain;
    private final LilihSubsystem lilihSubsystem;
    private SwerveDrivePoseEstimator estimator;

    private Rotation2d rotOffset;
    private Timer offsetTimer = new Timer();

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
                        drivetrain.getRawGyro(),
                        drivetrain.getModulePositions(),
                        initialPose);
        rotOffset = initialPose.getRotation().minus(drivetrain.getRawGyro());
    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public Pose3d getTagPose(int id) {
        return aprilTagFieldLayout.getTagPose(id).orElse(null);
    }

    private void updateEstimation() {
        estimator.update(drivetrain.getRawGyro(), drivetrain.getModulePositions());

            lilihSubsystem.addYawMeasurement(
                    drivetrain
                            .getRawGyro()
                            .plus(rotOffset != null ? rotOffset : new Rotation2d())
                            .getDegrees());
            if (lilihSubsystem.seeingAnything()) {
                PoseEstimate poseEstimate =
                        rotOffset != null
                                ? lilihSubsystem.getRobotPose_megaTag2()
                                : lilihSubsystem.getRobotPose();
                if (poseEstimate.rawFiducials != null
                        && poseEstimate.rawFiducials.length > 0
                        && poseEstimate.rawFiducials[0].ambiguity < .7) {
                    estimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
                    if (rotOffset == null) {
                        offsetTimer.start();
                        if (offsetTimer.get() > 0.5) setInitialPose(poseEstimate.pose);
                    }
                }
            }
    }

    @Override
    public void periodic() {
        updateEstimation();
    }

    @Override
    public LoggableInputs log() {
        poseEstimationLogAutoLogged.rotOffset = rotOffset;

        poseEstimationLogAutoLogged.combined = getPose();
        if (lilihSubsystem.seeingAnything())
            poseEstimationLogAutoLogged.limOnly = lilihSubsystem.getRobotPose().pose;
        poseEstimationLogAutoLogged.driveOnly = drivetrain.getPose();
        poseEstimationLogAutoLogged.pathPlannerPosy = pathPlannerPose;
        return poseEstimationLogAutoLogged;
    }

}
