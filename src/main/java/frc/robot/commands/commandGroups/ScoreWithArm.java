package frc.robot.commands.commandGroups;

import frc.robot.commands.algeePivotCommands.SetAlgeePivotCommand;
import frc.robot.commands.differentialArmCommands.SetArmPitchCommand;
import frc.robot.commands.driveCommands.CenterByButtonRingCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem.DifferentialArmPitch;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.ButtonRingController;
import frc.robot.utilities.UnInstantCommand;
import frc.robot.utilities.loggedComands.LoggedParallelCommandGroup;
import frc.robot.utilities.loggedComands.LoggedSequentialCommandGroup;

public class ScoreWithArm extends LoggedSequentialCommandGroup {

    ButtonRingController buttonRingController;

    public ScoreWithArm(
            AlgeePivotSubsystem algeePivotSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            ButtonRingController buttonRingController,
            DifferentialArmSubsystem differentialArmSubsystem,
            PoseEstimationSubsystem poseEstimationSubsystem,
            Drivetrain drivetrain) {

        this.buttonRingController = buttonRingController;

        addCommands(
                new SetAlgeePivotCommand(algeePivotSubsystem, AlgeePivotAngle.ZERO),
				new SetArmPitchCommand(differentialArmSubsystem, DifferentialArmPitch.NINETY),
				new PositionCoralCommand(elevatorSubsystem, differentialArmSubsystem, buttonRingController),
                new CenterByButtonRingCommand(poseEstimationSubsystem, drivetrain, buttonRingController),
				new ScoreCoralCommand(elevatorSubsystem, differentialArmSubsystem, buttonRingController));
    }

    @Override
    public void execute() {
        if (buttonRingController.getTagID() != 0 && buttonRingController.getxOffset() != 0)
            super.execute();
    }
}
