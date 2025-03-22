package frc.robot.commands.driveCommands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.light.LEDState;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.ButtonRingController;
import frc.robot.utilities.CenterDistance;

public class CenterByButtonRingCommand extends CenterOnTargetCommand {
    ButtonRingController buttonRingController;
    private final double clawffset = 0.1;

    public CenterByButtonRingCommand(
            PoseEstimationSubsystem poseEstimationSubsystem,
            Drivetrain drivetrain,
            ButtonRingController buttonRingController,
            CenterDistance centerDistance) {
        super(1, poseEstimationSubsystem, drivetrain);

        this.buttonRingController = buttonRingController;
        this.centerDistance = centerDistance;
    }

    @Override
    public void initialize() {
        target =
                placeTarget(
                        buttonRingController.getTagID(),
                        buttonRingController.getxOffset() - clawffset,
                        centerDistance);
        super.initialize();

        LEDState.centerRunning = true;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        LEDState.centerRunning = false;
    }

    @Override
    public double getTranslationTolerance() {
        return centerDistance.getTranslationTolerance();
    }

    @Override
    public double getRotationTolerance() {
        return centerDistance.getRotationTolerance();
    }
}
