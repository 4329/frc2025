package frc.robot.commands.driveCommands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.light.LEDState;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.ButtonRingController;

public class CenterByButtonRingCommand extends CenterOnTargetCommand {
    ButtonRingController buttonRingController;
    private final double clawOffset = 0.132;

    public CenterByButtonRingCommand(
            PoseEstimationSubsystem poseEstimationSubsystem,
            Drivetrain drivetrain,
            ButtonRingController buttonRingController) {
        super(1, poseEstimationSubsystem, drivetrain);

        this.buttonRingController = buttonRingController;
    }

    @Override
    public void initialize() {
        target =
                placeTarget(
                        buttonRingController.getTagID(), buttonRingController.getxOffset() - clawOffset);
        super.initialize();

        LEDState.centerRunning = true;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        LEDState.centerRunning = false;
    }
}
