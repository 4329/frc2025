package frc.robot.commands.driveCommands;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.light.LEDState;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.BetterPathfindingCommand;
import frc.robot.utilities.CenterDistance;
import frc.robot.utilities.loggedComands.LoggedCommandComposer;

import org.littletonrobotics.junction.Logger;

public class CenterOnTargetCommand extends LoggedCommandComposer {
    protected PoseEstimationSubsystem poseEstimationSubsystem;
    Drivetrain drivetrain;
    Command pathFind;
    protected Pose2d target;
    protected CenterDistance centerDistance;
    private final PathConstraints constraints =
            new PathConstraints(2, 1.0, Math.PI / 4, Math.PI / 16); // The constraints for this path.

    public CenterOnTargetCommand(
            int targetID, PoseEstimationSubsystem poseEstimationSubsystem, Drivetrain drivetrain) {
        this(targetID, poseEstimationSubsystem, drivetrain, 0, CenterDistance.INITIAL);
    }

    public CenterOnTargetCommand(
            int targetID,
            PoseEstimationSubsystem poseEstimationSubsystem,
            Drivetrain drivetrain,
            double xOffset,
            CenterDistance centerDistance) {
        this.poseEstimationSubsystem = poseEstimationSubsystem;
        this.drivetrain = drivetrain;

        target = placeTarget(targetID, xOffset, centerDistance);

        addRequirements(drivetrain);
    }

    public Pose2d placeTarget(int targetID, double xOffset, CenterDistance centerDistance) {
        Pose3d tmp = poseEstimationSubsystem.getTagPose(targetID);
        if (tmp == null) return null;

        target = tmp.toPose2d();
        Logger.recordOutput("target", target);
        target =
                new Pose2d(
                        target.getX()
                                + target.getRotation().getCos() * centerDistance.getzDist()
                                + Math.cos(target.getRotation().getRadians() + Math.PI / 2) * xOffset,
                        target.getY()
                                + target.getRotation().getSin() * centerDistance.getzDist()
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
                        getTranslationTolerance(),
                        getRotationTolerance(),
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
        return pathFind.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (pathFind != null) pathFind.cancel();

        drivetrain.resetKeepAngle();
        LEDState.centerRunning = false;
    }

    public double getTranslationTolerance() {
        return 0.002;
    }

    public double getRotationTolerance() {
        return 0.01;
    }
}
