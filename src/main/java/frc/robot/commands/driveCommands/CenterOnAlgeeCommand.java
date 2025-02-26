package frc.robot.commands.driveCommands;

import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.light.LEDState;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.ButtonRingController;

public class CenterOnAlgeeCommand extends CenterOnTargetCommand {

    ButtonRingController buttonRingController;

    public CenterOnAlgeeCommand(
            PoseEstimationSubsystem poseEstimationSubsystem,
            Drivetrain drivetrain,
            ButtonRingController buttonRingController) {
        super(0, poseEstimationSubsystem, drivetrain);
        this.buttonRingController = buttonRingController;
    }

    @Override
    public void initialize() {
        target = placeTarget(buttonRingController.getTagID(), 0);
        super.initialize();
        LEDState.centerRunning = true;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        LEDState.centerRunning = false;
    }
}
