package frc.robot.commands.driveCommands;

import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.ButtonRingController;

public class CenterByButtonRingCommand extends CenterOnTargetCommand {
  ButtonRingController buttonRingController;

  public CenterByButtonRingCommand(
      PoseEstimationSubsystem poseEstimationSubsystem,
      Drivetrain drivetrain,
      ButtonRingController buttonRingController) {
    super(0, poseEstimationSubsystem, drivetrain);

    this.buttonRingController = buttonRingController;
  }

  @Override
  public void initialize() {
    target = placeTarget(buttonRingController.getTagID(), buttonRingController.getxOffset());
    super.initialize();
  }
}
