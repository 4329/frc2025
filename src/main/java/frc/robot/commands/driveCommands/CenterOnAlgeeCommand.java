package frc.robot.commands.driveCommands;

import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.ButtonRingController;
import frc.robot.utilities.CenterDistance;

public class CenterOnAlgeeCommand extends CenterOnTargetCommand {

    ButtonRingController buttonRingController;

    private final double clawOffset = 0.156;

    public CenterOnAlgeeCommand(
            PoseEstimationSubsystem poseEstimationSubsystem,
            Drivetrain drivetrain,
            ButtonRingController buttonRingController,
            CenterDistance centerDistance) {
        super(0, poseEstimationSubsystem, drivetrain);
        this.buttonRingController = buttonRingController;
        this.centerDistance = centerDistance;
    }

    @Override
    public void initialize() {
        target = placeTarget(buttonRingController.getTagID(), clawOffset, centerDistance);
        super.initialize();
    }
}
