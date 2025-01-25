package frc.robot.commands.driveCommands;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.BetterPathfindingCommand;
import org.littletonrobotics.junction.Logger;

public class CenterOnTargetCommand extends Command {
  PoseEstimationSubsystem poseEstimationSubsystem;
  Drivetrain drivetrain;
  Command pathFind;
  protected Pose2d target;

  private final PathConstraints constraints =
      new PathConstraints(2, 3.0, Math.PI, Math.PI); // The constraints for this path.

  private final double zDist = .8;

  public CenterOnTargetCommand(
      int targetID, PoseEstimationSubsystem poseEstimationSubsystem, Drivetrain drivetrain) {
    this(targetID, poseEstimationSubsystem, drivetrain, 0);
  }

  public CenterOnTargetCommand(
      int targetID,
      PoseEstimationSubsystem poseEstimationSubsystem,
      Drivetrain drivetrain,
      double xOffset) {
    this.poseEstimationSubsystem = poseEstimationSubsystem;
    this.drivetrain = drivetrain;

    target = placeTarget(targetID, xOffset);

    Logger.recordOutput("target", target);
  }

  Pose2d placeTarget(int targetID, double xOffset) {
    Pose2d target = poseEstimationSubsystem.getTagPose(targetID).toPose2d();
    target =
        target.transformBy(
            new Transform2d(
                target.getRotation().getCos() * zDist + target.getRotation().getSin() * xOffset,
                target.getRotation().getSin() * zDist + target.getRotation().getCos() * xOffset,
                new Rotation2d(target.getRotation().getRadians() + Math.PI)));

    return target;
  }

  @Override
  public void initialize() {
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
