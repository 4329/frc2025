package frc.robot.commands.autoCommands;

import frc.robot.commands.algeePivotCommands.SetAlgeePivotCommand;
import frc.robot.commands.algeeWheelCommands.OuttakeAlgeeCommand;
import frc.robot.commands.differentialArmCommands.SetArmPitchCommand;
import frc.robot.commands.driveCommands.CenterOnTargetCommand;
import frc.robot.commands.elevatorCommands.SetElevatorCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import frc.robot.subsystems.AlgeeWheelSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem.DifferentialArmPitch;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.AprilTagUtil;
import frc.robot.utilities.CenterDistance;
import frc.robot.utilities.UnInstantCommand;
import frc.robot.utilities.loggedComands.LoggedSequentialCommandGroup;

public class PorcessorCommand extends LoggedSequentialCommandGroup {

    public PorcessorCommand(
            ElevatorSubsystem elevatorSubsystem,
            DifferentialArmSubsystem differentialArmSubsystem,
            AlgeePivotSubsystem algeePivotSubsystem,
            AlgeeWheelSubsystem algeeWheelSubsystem,
            PoseEstimationSubsystem poseEstimationSubsystem,
            Drivetrain drivetrain) {

        CenterOnTargetCommand center =
                new CenterOnTargetCommand(
                        AprilTagUtil::getPorcessor,
                        poseEstimationSubsystem,
                        drivetrain,
                        () -> 0.25,
                        CenterDistance.PORCESSOR);
        addCommands(
                new UnInstantCommand("calcInitial", center::calcInitial),
                new LoggedSequentialCommandGroup(
                                "DoArm",
                                new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.DIFFERENTIAL_ARM_OUT),
                                new SetArmPitchCommand(differentialArmSubsystem, DifferentialArmPitch.NINETY))
                        .onlyIfLog(
                                () ->
                                        differentialArmSubsystem.getPitchSetpoint()
                                                        != DifferentialArmPitch.NINETY.getRotation()
                                                && differentialArmSubsystem.pitchAtSetpoint()),
                new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.PORCESSOR),
                new SetAlgeePivotCommand(algeePivotSubsystem, AlgeePivotAngle.OUTFORCORAL),
                center,
                new OuttakeAlgeeCommand(algeeWheelSubsystem, 0.2));
    }
}
