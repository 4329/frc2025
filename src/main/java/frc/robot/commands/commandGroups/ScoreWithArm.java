package frc.robot.commands.commandGroups;

import java.lang.annotation.Target;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.differentialArmCommands.SetArmPitchCommand;
import frc.robot.commands.driveCommands.CenterByButtonRingCommand;
import frc.robot.commands.elevatorCommands.SetElevatorCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.ButtonRingController;
import frc.robot.utilities.UnInstantCommand;

public class ScoreWithArm {
    public static Command scoreWithArm(
            ElevatorSubsystem elevatorSubsystem,
            ButtonRingController buttonRingController,
            DifferentialArmSubsystem differentialArmSubsystem,
            PoseEstimationSubsystem poseEstimationSubsystem,
            Drivetrain drivetrain) {

        BooleanSupplier only = () -> buttonRingController.getLevel() != 0 && buttonRingController.getTagID() != 0;

        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                        new UnInstantCommand(() -> elevatorSubsystem.setSetpoint(
                                switch (buttonRingController.getLevel()) {
                                    case 2 -> ElevatorSubsystem.ElevatorPosition.L2;
                                    case 3 -> ElevatorSubsystem.ElevatorPosition.L3;
                                    case 4 -> ElevatorSubsystem.ElevatorPosition.L4;
                                    default -> ElevatorSubsystem.ElevatorPosition.L2;
                                })),
                        new SetArmPitchCommand(differentialArmSubsystem,
                                DifferentialArmSubsystem.DifferentialArmPitch.ONETHIRTYFIVE)
            ),

            new CenterByButtonRingCommand(poseEstimationSubsystem, drivetrain, buttonRingController),

            new SetArmPitchCommand(differentialArmSubsystem, DifferentialArmSubsystem.DifferentialArmPitch.NINETY)
        ).onlyIf(only).onlyWhile(only);
    }

}