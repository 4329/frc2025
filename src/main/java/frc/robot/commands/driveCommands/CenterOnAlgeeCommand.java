package frc.robot.commands.driveCommands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.ButtonRingController;

public class CenterOnAlgeeCommand extends CenterOnTargetCommand {

    ButtonRingController buttonRingController;
    private final double zDist = 1; // WIP change this later please

    private final double clawOffset = 0.156;

    public CenterOnAlgeeCommand(
            PoseEstimationSubsystem poseEstimationSubsystem,
            Drivetrain drivetrain,
            ButtonRingController buttonRingController) {
        super(0, poseEstimationSubsystem, drivetrain);
        this.buttonRingController = buttonRingController;
    }

    @Override
    public void initialize() {
        target = placeTarget(buttonRingController.getTagID(), clawOffset);
        super.initialize();
    }
}
