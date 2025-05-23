package frc.robot.subsystems;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.model.PoseEstimationLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.subsystems.light.LEDState;
import frc.robot.subsystems.lilih.LilihSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.AABB;
import frc.robot.utilities.LimelightHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.List;
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
    private Pose2d pathPlannerTarget = new Pose2d();
    private List<Pose2d> pathPlannerposes = new ArrayList<>();

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
                    pathPlannerTarget = pose != null ? pose : new Pose2d();
                });

        PathPlannerLogging.setLogActivePathCallback(
                (poses) -> {
                    field.getObject("path").setPoses(poses);
                    pathPlannerposes = poses != null ? poses : new ArrayList<>();
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
    }

    private void setRotOffset(Pose2d initialPose) {
        setInitialPose(initialPose);
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
            if (poseEstimate != null
                    && poseEstimate.rawFiducials != null
                    && poseEstimate.rawFiducials.length > 0
                    && poseEstimate.rawFiducials[0] != null
                    && poseEstimate.rawFiducials[0].ambiguity < .7
                    && poseEstimate.rawFiducials[0].ta > 0.003) {
                estimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
                if (rotOffset == null) {
                    offsetTimer.start();
                    if (offsetTimer.get() > 0.5) setRotOffset(poseEstimate.pose);
                }
            }
        }
    }

    private Pose2d getAlliancedPose() {
        Pose2d pose = getPose();
        if (Alliance.Red.equals(DriverStation.getAlliance().orElse(null)))
            pose =
                    new Pose2d(
                            8.7 - (pose.getX() - 8.7), 8 - pose.getY(), pose.getRotation().minus(Rotation2d.kPi));
        return pose;
    }

    final AABB barge = new AABB(8.75, 4, 1.15, 4);
    final AABB porcessor = new AABB(6.18, 0, 0.75, 0.75);
    final AABB hpStation1 = new AABB(0.55, 0.55, 0.975, 0.825);
    final AABB hpStation2 = new AABB(0.55, 7.45, 0.975, 0.825);
    final Translation2d reef = new Translation2d(4.48, 4.03);

    @Override
    public void periodic() {
        updateEstimation();
        Pose2d pose = getAlliancedPose();
        AABB robot =
                new AABB(
                        pose.getX(), pose.getY(), Units.inchesToMeters(29) / 2, Units.inchesToMeters(30) / 2);
        LEDState.byBarge = robot.intersectingAABB(barge);
        LEDState.byHpStation = hpStation1.intersectingAABB(robot) || hpStation2.intersectingAABB(robot);
        LEDState.byPorcessor = robot.intersectingAABB(porcessor);
        LEDState.byReef = pose.getTranslation().getDistance(reef) < 1.55;
    }

    public void resetRotOffset() {
        rotOffset = null;
    }

    @Override
    public LoggableInputs log() {
        poseEstimationLogAutoLogged.rotOffset = rotOffset;

        poseEstimationLogAutoLogged.combined = getPose();
        if (lilihSubsystem.seeingAnything())
            poseEstimationLogAutoLogged.limOnly = lilihSubsystem.getRobotPose().pose;
        poseEstimationLogAutoLogged.driveOnly = drivetrain.getPose();

        poseEstimationLogAutoLogged.pathPlannerPosy = pathPlannerPose;
        poseEstimationLogAutoLogged.pathPlannerTarget = pathPlannerTarget;
        poseEstimationLogAutoLogged.pathPlannerPoses = pathPlannerposes.toArray(new Pose2d[] {});
        return poseEstimationLogAutoLogged;
    }
}
