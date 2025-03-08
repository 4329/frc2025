package frc.robot.commands.autoCommands;

import frc.robot.commands.driveCommands.CenterOnTargetCommand;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.AprilTagUtil;
import frc.robot.utilities.CenterDistance;

public class CenterInAutoCommand extends CenterOnTargetCommand {

    boolean right;
    int num;

    public CenterInAutoCommand(
            PoseEstimationSubsystem poseEstimationSubsystem,
            Drivetrain drivetrain,
            int num,
            boolean right,
            CenterDistance centerDistance) {
        super(1, poseEstimationSubsystem, drivetrain);
        this.num = num;
        this.right = right;
        this.centerDistance = centerDistance;
    }

    @Override
    public void initialize() {
        target =
                placeTarget(
                        AprilTagUtil.getReef(num),
                        PoseEstimationSubsystem.OFFSET * (right ? 1 : -1),
                        centerDistance);
        super.initialize();
    }
}
