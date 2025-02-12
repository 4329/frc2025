package frc.robot.commands.visionCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.lilih.LilihSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;

public class LimDriveSetCommand extends Command {

    private LilihSubsystem limlihSubsystem;
    private Drivetrain drivetrain;
    private PoseEstimationSubsystem poseEstimationSubsystem;

    public LimDriveSetCommand(
            LilihSubsystem limlihSubsystem,
            Drivetrain drivetrain,
            PoseEstimationSubsystem poseEstimationSubsystem) {

        this.limlihSubsystem = limlihSubsystem;
        this.drivetrain = drivetrain;
        this.poseEstimationSubsystem = poseEstimationSubsystem;
    }

    @Override
    public void initialize() {
        Pose2d carl = limlihSubsystem.getRobotPose().pose;

        if (limlihSubsystem.seeingAnything() && !limlihSubsystem.getRobotPose().equals(new Pose2d()))
            drivetrain.resetOdometry(carl);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
