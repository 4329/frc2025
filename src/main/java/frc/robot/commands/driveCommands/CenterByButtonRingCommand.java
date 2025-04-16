package frc.robot.commands.driveCommands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.ButtonRingController;
import frc.robot.utilities.CenterDistance;

public class CenterByButtonRingCommand extends CenterOnTargetCommand {
    static GenericEntry adf = Shuffleboard.getTab("adsjoijdsa").add("xnoivewoiewoi", 0.17).getEntry();
    private ButtonRingController buttonRingController;

    // private final double clawffset = 0.14;

    public CenterByButtonRingCommand(
            PoseEstimationSubsystem poseEstimationSubsystem,
            Drivetrain drivetrain,
            ButtonRingController buttonRingController,
            CenterDistance centerDistance) {
        super(
                buttonRingController::getTagID,
                poseEstimationSubsystem,
                drivetrain,
                () -> buttonRingController.getxOffset() - adf.getDouble(0),
                centerDistance);
        this.buttonRingController = buttonRingController;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public void calcInitial() {
        if (buttonRingController.getLevel() == 4 && CenterDistance.SCORING.equals(centerDistance)) {
            centerDistance = CenterDistance.SCORING_SMALL;
        }
        super.calcInitial();
    }
}
