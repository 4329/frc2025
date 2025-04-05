package frc.robot.commands.commandGroups;

import frc.robot.commands.algeePivotCommands.SetAlgeePivotCommand;
import frc.robot.commands.driveCommands.CenterByButtonRingCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.ButtonRingController;
import frc.robot.utilities.CenterDistance;
import frc.robot.utilities.loggedComands.LoggedSequentialCommandGroup;

public class ScoreWithArm extends LoggedSequentialCommandGroup {

    ButtonRingController buttonRingController;
    Drivetrain drivetrain;

    public ScoreWithArm(
            AlgeePivotSubsystem algeePivotSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            ButtonRingController buttonRingController,
            DifferentialArmSubsystem differentialArmSubsystem,
            PoseEstimationSubsystem poseEstimationSubsystem,
            Drivetrain drivetrain) {

        this.buttonRingController = buttonRingController;
        this.drivetrain = drivetrain;

        addCommands(
                new SetAlgeePivotCommand(algeePivotSubsystem, AlgeePivotAngle.OUTFORCORAL),
                new PositionCoralCommand(elevatorSubsystem, differentialArmSubsystem, buttonRingController),
                // new CenterByButtonRingCommand(
                //        poseEstimationSubsystem,
                //        drivetrain,
                //        buttonRingController,
                //        CenterDistance.INITIAL) // .untilLog(positionCoral::isFinished)
                // ),
                new CenterByButtonRingCommand(
                        poseEstimationSubsystem, drivetrain, buttonRingController, CenterDistance.SCORING),
                new ScoreCoralCommand(
                        elevatorSubsystem,
                        differentialArmSubsystem,
                        buttonRingController,
                        algeePivotSubsystem));
    }

    @Override
    public void execute() {
        if (buttonRingController.getTagID() != 0 && buttonRingController.getxOffset() != 0)
            super.execute();
        else drivetrain.stop();
    }
}
