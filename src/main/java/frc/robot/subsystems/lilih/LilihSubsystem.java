package frc.robot.subsystems.lilih;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.model.LilihLog;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.utilities.LimelightHelpers.PoseEstimate;
import frc.robot.utilities.MathUtils;
import org.littletonrobotics.junction.Logger;

public class LilihSubsystem extends SubsystemBase {

    private final String limelightHelpNetworkTableName;

    LimelightTarget_Fiducial[] limelightResults;
    LilihSocket lilihSocket;

    GenericEntry connected =
            Shuffleboard.getTab("RobotData")
                    .add("Lilih Connected", false)
                    .withPosition(7, 2)
                    .withSize(3, 2)
                    .getEntry();

    GenericEntry seeTag =
            Shuffleboard.getTab("RobotData")
                    .add("Seeing tag", false)
                    .withPosition(10, 2)
                    .withSize(2, 2)
                    .getEntry();

    private LilihLog lilihLog;

    public LilihSubsystem(LilihSocket lilihSocket) {
        this.lilihSocket = lilihSocket;
        lilihLog = new LilihLog();
        this.limelightHelpNetworkTableName = "";
    }

    public LilihSubsystem(int ip, String limelightHelpNetworkTableName) {
        lilihLog = new LilihLog();
        lilihSocket = new LilihSocket(ip);
        this.limelightHelpNetworkTableName = limelightHelpNetworkTableName;
        switchPipeline(0);
    }

    public boolean cameraConnected() {
        return lilihSocket.isConnected();
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

    public PoseEstimate getRobotPose() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightHelpNetworkTableName);
    }

    /**
     * Pose calculated with all markers
     *
     * @return Pose
     */
    public PoseEstimate getRobotPose_megaTag2() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightHelpNetworkTableName);
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
        return limelightResults != null && limelightResults.length > 0;
    }

    private void updateInputs() {
        for (int i = 0; i < LilihLog.NUM_TAGS; i++) {
            lilihLog.tags[i].tV = getTargetVisible(i + 1);
            if (lilihLog.tags[i].tV) {
                LimelightTarget_Fiducial fiducial = getFiducial(i + 1);
                lilihLog.tags[i].tX = fiducial.tx;
                lilihLog.tags[i].tY = fiducial.ty;
                lilihLog.tags[i].tA = fiducial.ta;
            } else {
                lilihLog.tags[i].tX = 0;
                lilihLog.tags[i].tY = 0;
                lilihLog.tags[i].tA = 0;
            }
        }
        lilihLog.limlihConnected = cameraConnected();
        connected.setBoolean(cameraConnected());
        seeTag.setBoolean(seeingAnything());

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
        if (cameraConnected()) {
            limelightResults = lilihSocket.getResults().targets_Fiducials;
        }

        updateInputs();
    }

    public double getTargetX(int id) {
        return getFiducial(id).tx;
    }

    public void addYawMeasurement(double yaw) {
        LimelightHelpers.SetRobotOrientation(limelightHelpNetworkTableName, yaw, 0, 0, 0, 0, 0);
    }
}
