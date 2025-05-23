package frc.robot.commands.commandGroups;

import frc.robot.commands.algeePivotCommands.SetAlgeePivotCommand;
import frc.robot.commands.differentialArmCommands.SetArmPitchCommand;
import frc.robot.commands.elevatorCommands.SetElevatorCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem.DifferentialArmPitch;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.utilities.loggedComands.LoggedSequentialCommandGroup;

public class HappyResetCommand extends LoggedSequentialCommandGroup {

    public HappyResetCommand(
            DifferentialArmSubsystem differentialArmSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            AlgeePivotSubsystem algeePivotSubsystem) {

        addCommands(
                new LoggedSequentialCommandGroup(
                                "StoreAlgee",
                                new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.DIFFERENTIAL_ARM_OUT)
                                        .onlyIfLog(
                                                () ->
                                                        differentialArmSubsystem.getPitchSetpoint()
                                                                        != DifferentialArmPitch.NINETY.getRotation()
                                                                || !differentialArmSubsystem.pitchAtSetpoint()),
                                new SetArmPitchCommand(
                                        differentialArmSubsystem, DifferentialArmSubsystem.DifferentialArmPitch.NINETY),
                                new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.ALGEE_CLAW_OUT),
                                new SetAlgeePivotCommand(algeePivotSubsystem, AlgeePivotAngle.ZERO))
                        .onlyIfLog(
                                () ->
                                        algeePivotSubsystem.getSetpoint() != AlgeePivotAngle.ZERO.angle
                                                || !algeePivotSubsystem.atSetpoint()),
                new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.DIFFERENTIAL_ARM_OUT)
                        .onlyIfLog(
                                () ->
                                        differentialArmSubsystem.getPitchSetpoint()
                                                        != DifferentialArmPitch.STORAGE.getRotation()
                                                || !differentialArmSubsystem.pitchAtSetpoint()),
                new SetArmPitchCommand(
                        differentialArmSubsystem, DifferentialArmSubsystem.DifferentialArmPitch.STORAGE),
                new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.ZERO));
    }
}
