package frc.robot.commands.autoCommands;

import frc.robot.commands.driveCommands.CenterOnTargetCommand;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.AprilTagUtil;

public class CenterInAutoCommand extends CenterOnTargetCommand {

    boolean right;
    int num;
    double zDist;

    public CenterInAutoCommand(
            PoseEstimationSubsystem poseEstimationSubsystem,
            Drivetrain drivetrain,
            int num,
            boolean right) {
        this(
                poseEstimationSubsystem,
                drivetrain,
                num,
                right,
                DifferentialArmSubsystem.ARM_LENGTH_CORAL_CENTER);
    }

    public CenterInAutoCommand(
            PoseEstimationSubsystem poseEstimationSubsystem,
            Drivetrain drivetrain,
            int num,
            boolean right,
            double zDist) {
        super(1, poseEstimationSubsystem, drivetrain);
        this.num = num;
        this.right = right;
        this.zDist = zDist;
    }

    @Override
    public void initialize() {
        target =
                placeTarget(
                        AprilTagUtil.getReef(num), PoseEstimationSubsystem.OFFSET * (right ? 1 : -1), zDist);
        super.initialize();
    }
}
