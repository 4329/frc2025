package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.algeePivotCommands.SetAlgeePivotCommand;
import frc.robot.commands.differentialArmCommands.SetArmPitchCommand;
import frc.robot.commands.driveCommands.CenterByButtonRingCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.ButtonRingController;
import frc.robot.utilities.NotFinalSequentialCommandGroup;
import frc.robot.utilities.UnInstantCommand;

public class ScoreWithArm extends NotFinalSequentialCommandGroup {

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
                new ParallelCommandGroup(
                        new UnInstantCommand(
                                        () ->
                                                elevatorSubsystem.setSetpoint(
                                                        switch (buttonRingController.getLevel()) {
                                                            case 2 -> ElevatorSubsystem.ElevatorPosition.L2;
                                                            case 3 -> ElevatorSubsystem.ElevatorPosition.L3;
                                                            case 4 -> ElevatorSubsystem.ElevatorPosition.L4;
                                                            default -> ElevatorSubsystem.ElevatorPosition.L2;
                                                        }))
                                .until(() -> elevatorSubsystem.atSetpoint()),
                        new SetArmPitchCommand(
                                differentialArmSubsystem,
                                DifferentialArmSubsystem.DifferentialArmPitch.ONETHIRTYFIVE)),
                new CenterByButtonRingCommand(poseEstimationSubsystem, drivetrain, buttonRingController),
                new SetArmPitchCommand(
                        differentialArmSubsystem, DifferentialArmSubsystem.DifferentialArmPitch.NINETY));
    }

    @Override
    public void execute() {
        if (buttonRingController.getTagID() != 0 && buttonRingController.getxOffset() != 0)
            super.execute();
    }
}
