package frc.robot.subsystems;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Model.PoseEstimationLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class PoseEstimationSubsystem extends SubsystemBase implements LoggedSubsystem {

  PoseEstimationLogAutoLogged poseEstimationLogAutoLogged;

  private final Drivetrain drivetrain;
  private final VisionSubsystem visionSubsystem;
  private SwerveDrivePoseEstimator estimator;
  private final double pathPlannerFieldWidth = 8.21;
  private final double pathPlannerFieldLength = 16.54;
  private Field2d field = new Field2d();
  private Pose2d pathPlannerPose = new Pose2d();

  private final double shootDexerZ = 0.484;
  private final double shootDexerX = -0.115;
  private final double shooterYawOffset = -0.1;

  private Pose2d initialPose;

  public PoseEstimationSubsystem(
      Drivetrain drivetrain, VisionSubsystem visionSubsystem) {
    this.visionSubsystem = visionSubsystem;
    this.drivetrain = drivetrain;

    poseEstimationLogAutoLogged = new PoseEstimationLogAutoLogged();
    estimator =
        new SwerveDrivePoseEstimator(
            Constants.DriveConstants.kDriveKinematics,
            drivetrain.getGyro(),
            drivetrain.getModulePositions(),
            drivetrain.getPose());

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
            transformPathPlannerToField(initialPose));
  }

  public Pose2d getPose() {
    return estimator.getEstimatedPosition();
  }

  private void updateEstimation() {
    estimator.update(drivetrain.getGyro(), drivetrain.getModulePositions());
    // if (visionSubsystem.seeingAnything()) {
    //   estimator.addVisionMeasurement(visionSubsystem.getRobotPose(), Timer.getFPGATimestamp());
    // }
  }

  @Override
  public void periodic() {
    updateEstimation();
  }

  @Override
  public LoggableInputs log() {
    poseEstimationLogAutoLogged.combined = transformFieldToAdvantageKit(getPose());
    poseEstimationLogAutoLogged.limOnly =
        transformFieldToAdvantageKit(visionSubsystem.getRobotPose());
    poseEstimationLogAutoLogged.driveOnly = transformFieldToAdvantageKit(drivetrain.getPose());
    poseEstimationLogAutoLogged.pathPlannerPosy = pathPlannerPose;
    Logger.recordOutput("zero", new Pose2d());
    return poseEstimationLogAutoLogged;
  }

  private Pose2d transformFieldToAdvantageKit(Pose2d pose) {
    return new Pose2d(
        pose.getX() + (pathPlannerFieldWidth / 2),
        pose.getY() + (pathPlannerFieldLength / 2),
        pose.getRotation());
  }

  private Pose2d transformFieldToPathPlanner(Pose2d pose) {
    return new Pose2d(
        pose.getX() + (pathPlannerFieldLength / 2),
        pose.getY() + (pathPlannerFieldWidth / 2),
        pose.getRotation());
  }

  private Pose2d transformPathPlannerToField(Pose2d pose) {
    return new Pose2d(
        pose.getX() - (pathPlannerFieldLength / 2),
        pose.getY() - (pathPlannerFieldWidth / 2),
        pose.getRotation());
  }

  public Pose2d getPathPlannerStuff() {
    return transformFieldToPathPlanner(getPose());
  }
}
