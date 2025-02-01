package frc.robot.subsystems.lilih;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.model.LilihLog;
import frc.robot.utilities.AprilTagUtil;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.utilities.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.utilities.LimelightHelpers.PoseEstimate;
import frc.robot.utilities.MathUtils;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class LilihSubsystem extends SubsystemBase {

  double[] hrm;
  private final String limelightHelpNetworkTableName;

  LimelightTarget_Fiducial[] limelightResults;
  LimelightTarget_Detector limelightResultsDetector;

  public LimelightTarget_Detector getLimelightResultsDetector() {
    return limelightResultsDetector;
  }

  LilihSocket lilihSocket;

  private GenericEntry zGE;
  private GenericEntry sight;
  private GenericEntry sighttwo;

  private Timer timer;
  private CheckLilihCommand checkLimelightCommand;

  private LilihLog lilihLog;

  public LilihSubsystem(int ip, String limelightHelpNetworkTableName) {
    timer = new Timer();
    timer.start();
    this.checkLimelightCommand = new CheckLilihCommand(ip);

    zGE = Shuffleboard.getTab("shoot").add("zPose", 0).getEntry();
    sight =
        Shuffleboard.getTab("RobotData")
            .add("Seeing Speaker", false)
            .withPosition(4, 0)
            .withSize(9, 2)
            .withProperties(Map.of("Color when true", "#0000FF", "Color when false", "#000000"))
            .getEntry();

    sighttwo =
        Shuffleboard.getTab("RobotData")
            .add("Need Elevator?", false)
            .withPosition(4, 2)
            .withSize(9, 1)
            .withProperties(Map.of("Color when true", "#00FFFF", "Color when false", "#000000"))
            .getEntry();

    lilihLog = new LilihLog();
    lilihSocket = new LilihSocket(ip);
    this.limelightHelpNetworkTableName = limelightHelpNetworkTableName;
    switchPipeline(0);
  }

  public boolean cameraConnected() {
    return checkLimelightCommand.isConnected();
  }

  public boolean getTargetVisible(int id) {
    if (limelightResults == null) {
      return false;
    }
    for (LimelightTarget_Fiducial LIMGHT : limelightResults) {
      if (LIMGHT.fiducialID == id) {
        return true;
      }
    }
    return false;
  }

  /**
   * Pose calculated with a single marker
   *
   * @param id
   * @return Pose
   */
  public Pose2d getRobotFieldPoseByTag(int id) {

    return getFiducial(id).getRobotPose_FieldSpace2D();
  }

  /**
   * Pose calculated with all markers
   *
   * @return Pose
   */
  public PoseEstimate getRobotPose() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightHelpNetworkTableName);
  }

  public Pose3d getRobotPoseInTargetSpace(int id) {
    return getFiducial(id).getRobotPose_TargetSpace();
  }

  public Pose3d getTargetPoseInRobotSpace(int id) {

    LimelightTarget_Fiducial limetarget = getFiducial(id);
    if (limetarget != null) {
      return limetarget.getTargetPose_RobotSpace();
    }
    return null;
  }

  public Pose3d getTargetSpacePose(int id) {

    return getFiducial(id).getRobotPose_TargetSpace();
  }

  public void switchPipeline(int pipeline) {

    LimelightHelpers.setPipelineIndex(limelightHelpNetworkTableName, pipeline);
  }

  public double getPipeline() {

    return LimelightHelpers.getCurrentPipelineIndex(limelightHelpNetworkTableName);
  }

  private LimelightTarget_Fiducial getFiducial(int id) {
    if (limelightResults == null) return null;
    for (LimelightTarget_Fiducial LIMGHT : limelightResults) {
      if (LIMGHT.fiducialID == id) {
        return LIMGHT;
      }
    }
    return null;
  }

  public boolean seeingAnything() {
    if (limelightResults != null && limelightResults.length > 0) {
      return true;
    } else {
      return false;
    }
  }

  private void updateInputs() {
    for (int i = 0; i < 16; i++) {
      lilihLog.tags[i].tV = getTargetVisible(i);
      if (lilihLog.tags[i].tV) {
        LimelightTarget_Fiducial fiducial = getFiducial(i);
        lilihLog.tags[i].tX = fiducial.tx;
        lilihLog.tags[i].tY = fiducial.ty;
        lilihLog.tags[i].relativePose = fiducial.getCameraPose_TargetSpace();
        Logger.recordOutput(
            "acl/" + i,
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField).getTagPose(i).get());
      } else {
        lilihLog.tags[i].tX = 0;
        lilihLog.tags[i].tY = 0;
        lilihLog.tags[i].relativePose = new Pose3d();
      }
    }
    lilihLog.limlihConnected = cameraConnected();

    Logger.processInputs("Lilihsubsystem", lilihLog);
  }

  public LimelightTarget_Fiducial limelightTarget_Fiducial(int id) {
    for (LimelightTarget_Fiducial LIMGHT : limelightResults) {
      if (LIMGHT.fiducialID == id) {
        return LIMGHT;
      }
    }
    return null;
  }

  public Pose2d getTargetPoseInFieldSpace(int id) {
    return getRobotFieldPoseByTag(id)
        .plus(MathUtils.pose2dToTransform2d(getTargetPoseInRobotSpace(id).toPose2d()));
  }

  @Override
  public void periodic() {
    if (checkLimelightCommand.isConnected()) {
      limelightResults = lilihSocket.getResults().targets_Fiducials;
      if (lilihSocket.getResults().targets_Detector != null
          && lilihSocket.getResults().targets_Detector.length > 0) {
        limelightResultsDetector = lilihSocket.getResults().targets_Detector[0];
      } else {
        limelightResultsDetector = null;
      }
    }

    updateInputs();

    occasionalCheck();
  }

  private void occasionalCheck() {
    if (timer.hasElapsed(4) && !checkLimelightCommand.isScheduled()) {
      checkLimelightCommand.schedule();
      timer.reset();
    }
  }

  public double getTargetX(int id) {
    return getFiducial(id).tx;
  }
}
