package frc.robot.commands.driveCommands;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem.DifferentialArmPitch;
import frc.robot.subsystems.light.LEDState;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.BetterPathfindingCommand;
import org.littletonrobotics.junction.Logger;

public class CenterOnTargetCommand extends Command {
    PoseEstimationSubsystem poseEstimationSubsystem;
    Drivetrain drivetrain;
    Command pathFind;
    protected Pose2d target;

    private final PathConstraints constraints =
            new PathConstraints(2, 1.0, Math.PI / 4, Math.PI / 16); // The constraints for this path.

    double zDist = DifferentialArmSubsystem.ARM_LENGTH_CORAL_CENTER;

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
    }

    Pose2d placeTarget(int targetID, double xOffset) {
        Pose3d tmp = poseEstimationSubsystem.getTagPose(targetID);
        if (tmp == null) return null;

        target = tmp.toPose2d();
        Logger.recordOutput("target", target);
        target =
                new Pose2d(
                        target.getX()
                                + target.getRotation().getCos() * zDist
                                + Math.cos(target.getRotation().getRadians() + Math.PI / 2) * xOffset,
                        target.getY()
                                + target.getRotation().getSin() * zDist
                                + Math.sin(target.getRotation().getRadians() + Math.PI / 2) * xOffset,
                        new Rotation2d(target.getRotation().getRadians() + Math.PI));
        Logger.recordOutput("posy", target);

        return target;
    }

    @Override
    public void initialize() {
        if (target == null) return;

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

        LEDState.centerRunning = true;
    }

    @Override
    public boolean isFinished() {
        if (target == null) return true;

        Logger.recordOutput(
                "off",
                poseEstimationSubsystem.getPose().getTranslation().getDistance(target.getTranslation()));
        Logger.recordOutput(
                "off2",
                Math.abs(
                        poseEstimationSubsystem
                                .getPose()
                                .getRotation()
                                .minus(target.getRotation())
                                .getRadians()));

        return poseEstimationSubsystem.getPose().getTranslation().getDistance(target.getTranslation())
                        < 0.002
                && Math.abs(
                                poseEstimationSubsystem
                                        .getPose()
                                        .getRotation()
                                        .minus(target.getRotation())
                                        .getRadians())
                        < 0.01;
    }

    @Override
    public void end(boolean interrupted) {
        if (pathFind != null) pathFind.cancel();

        LEDState.centerRunning = false;
    }
}
