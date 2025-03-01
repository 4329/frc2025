package frc.robot.commands.autoCommands;

import frc.robot.commands.driveCommands.CenterOnTargetCommand;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.AprilTagUtil;

public class CenterInAutoCommand extends CenterOnTargetCommand {

    boolean right;
    int num;

    public CenterInAutoCommand(
            PoseEstimationSubsystem poseEstimationSubsystem,
            Drivetrain drivetrain,
            int num,
            boolean right) {
        super(1, poseEstimationSubsystem, drivetrain);
        this.num = num;
        this.right = right;
    }

    @Override
    public void initialize() {
        target =
                placeTarget(AprilTagUtil.getReef(num), PoseEstimationSubsystem.OFFSET * (right ? 1 : -1), DifferentialArmSubsystem.ARM_LENGTH_CORAL_CENTER);
        super.initialize();
    }
}
