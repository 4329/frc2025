package frc.robot.commands.driveCommands;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PoseEstimationSubsystem;
<<<<<<< HEAD
=======
import frc.robot.subsystems.lilih.LilihSubsystem;
>>>>>>> 2109cc6 (aahaahhhhhhhhhhh)
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
<<<<<<< HEAD
import frc.robot.utilities.BetterPathfindingCommand;
=======
import frc.robot.utilities.ShuffledPIDController;
>>>>>>> 0cfc1eb (SHuffledPID)
import org.littletonrobotics.junction.Logger;

public class CenterOnTargetCommand extends Command {
<<<<<<< HEAD
  int targetID;
  PoseEstimationSubsystem poseEstimationSubsystem;
  Drivetrain drivetrain;
  Command pathFind;
  Pose2d target;
=======
  private final PoseEstimationSubsystem poseEstimationSubsystem;
  private final LilihSubsystem lilihSubsystem;
  private final Drivetrain drivetrain;
  private int targetId;
>>>>>>> 2109cc6 (aahaahhhhhhhhhhh)

<<<<<<< HEAD
  private final PathConstraints constraints =
      new PathConstraints(2, 3.0, Math.PI, Math.PI); // The constraints for this path.

  private final double zDist = .8;
=======
  private final ShuffledPIDController rotationPID;
  private final ShuffledPIDController xPID;
  private final ShuffledPIDController yPID;
>>>>>>> 0cfc1eb (SHuffledPID)

  private Pose2d initOdometry;
  private Pose2d newOdometry;

  private final double zTarget = -1;

  public CenterOnTargetCommand(
<<<<<<< HEAD
      int targetID, PoseEstimationSubsystem poseEstimationSubsystem, Drivetrain drivetrain) {
    this(targetID, poseEstimationSubsystem, drivetrain, 0);
  }
=======
      PoseEstimationSubsystem poseEstimationSubsystem,
      LilihSubsystem lilihSubsystem,
      Drivetrain m_drivetrain,
      int targetId) {
    this.poseEstimationSubsystem = poseEstimationSubsystem;
    this.lilihSubsystem = lilihSubsystem;
    this.drivetrain = m_drivetrain;
    this.targetId = targetId;
>>>>>>> 2109cc6 (aahaahhhhhhhhhhh)

  public CenterOnTargetCommand(
      int targetID,
      PoseEstimationSubsystem poseEstimationSubsystem,
      Drivetrain drivetrain,
      double xOffset) {
    this.targetID = targetID;
    this.poseEstimationSubsystem = poseEstimationSubsystem;
    this.drivetrain = drivetrain;

    target = poseEstimationSubsystem.getTagPose(targetID).toPose2d();
    target =
        target.transformBy(
            new Transform2d(
                target.getRotation().getCos() * zDist + target.getRotation().getSin() * xOffset,
                target.getRotation().getSin() * zDist + target.getRotation().getCos() * xOffset,
                new Rotation2d(target.getRotation().getRadians() + Math.PI)));

    Logger.recordOutput("target", target);
  }

  @Override
  public void initialize() {
<<<<<<< HEAD
    pathFind =
        new BetterPathfindingCommand(
            target,
            constraints,
            0,
            poseEstimationSubsystem::getPose,
            drivetrain::getChassisSpeed,
            (chassisSpeeds, a) -> drivetrain.setModuleStates(chassisSpeeds),
            Constants.AutoConstants.ppHolonomicDriveController,
            Constants.AutoConstants.config,
            drivetrain);
    pathFind.schedule();
=======
    if (!lilihSubsystem.cameraConnected() || !lilihSubsystem.getTargetVisible(targetId)) {
      return;
    }

    rotationPID.reset();
    drivetrain.stop();
    initOdometry = drivetrain.getPose();
    newOdometry = lilihSubsystem.getTargetPoseInFieldSpace(targetId);
    drivetrain.resetOdometry(
        new Pose2d(
            newOdometry.getTranslation(),
            new Rotation2d(
                lilihSubsystem
                    .getTargetPoseInRobotSpace(targetId)
                    .getRotation()
                    .getMeasureY()
                    .in(Units.Radians))));

    Pose2d tagPose = poseEstimationSubsystem.getTagPose(targetId).toPose2d();
    xPID.setSetpoint(tagPose.getX() + zTarget * Math.cos(tagPose.getRotation().getRadians()));
    yPID.setSetpoint(tagPose.getX() + zTarget * Math.sin(tagPose.getRotation().getRadians()));
    rotationPID.setSetpoint(-tagPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    if (lilihSubsystem.cameraConnected() && lilihSubsystem.getTargetVisible(targetId)) {

      Logger.recordOutput("taginfieldspace", lilihSubsystem.getTargetPoseInFieldSpace(targetId));

      double rotationCalc =
          rotationPID.calculate(
              lilihSubsystem
                  .getTargetPoseInRobotSpace(targetId)
                  .getRotation()
                  .getMeasureY()
                  .in(Units.Radians));
      double xCalc = xPID.calculate(lilihSubsystem.getTargetPoseInRobotSpace(targetId).getX());
      double yCalc = -yPID.calculate(lilihSubsystem.getTargetPoseInRobotSpace(targetId).getZ());

      if (rotationCalc > Constants.DriveConstants.kMaxAngularSpeed) {
        rotationCalc = Constants.DriveConstants.kMaxAngularSpeed;
      } else if (rotationCalc < -Constants.DriveConstants.kMaxAngularSpeed) {
        rotationCalc = -Constants.DriveConstants.kMaxAngularSpeed;
      } else if (rotationPID.atSetpoint()) {
        rotationCalc = 0;
      }

      if (xPID.atSetpoint()) xCalc = 0;
      if (yPID.atSetpoint()) yCalc = 0;

      Logger.recordOutput("xCalc", xCalc);
      Logger.recordOutput("yCalc", yCalc);
      Logger.recordOutput("rotationCalc", rotationCalc);

      drivetrain.drive(yCalc, xCalc, rotationCalc, true);
    } else {
      drivetrain.stop();
    }
>>>>>>> 2109cc6 (aahaahhhhhhhhhhh)
  }

  @Override
  public boolean isFinished() {
    return poseEstimationSubsystem.getPose().getTranslation().getDistance(target.getTranslation())
            < 0.1
        && Math.abs(
                poseEstimationSubsystem.getPose().getRotation().getRadians()
                    - target.getRotation().getRadians())
            < 0.1;
  }

  @Override
  public void end(boolean interrupted) {
    pathFind.cancel();
  }
}
