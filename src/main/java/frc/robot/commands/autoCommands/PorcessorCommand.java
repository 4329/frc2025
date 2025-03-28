package frc.robot.commands.autoCommands;

import frc.robot.commands.algeePivotCommands.SetAlgeePivotCommand;
import frc.robot.commands.algeeWheelCommands.OuttakeAlgeeCommand;
import frc.robot.commands.differentialArmCommands.SetArmPitchCommand;
import frc.robot.commands.elevatorCommands.SetElevatorCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import frc.robot.subsystems.AlgeeWheelSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem.DifferentialArmPitch;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.utilities.loggedComands.LoggedSequentialCommandGroup;

public class PorcessorCommand extends LoggedSequentialCommandGroup {

    public PorcessorCommand(
            ElevatorSubsystem elevatorSubsystem,
            DifferentialArmSubsystem differentialArmSubsystem,
            AlgeePivotSubsystem algeePivotSubsystem,
            AlgeeWheelSubsystem algeeWheelSubsystem) {

        addCommands(
                new LoggedSequentialCommandGroup(
                                "DoArm",
                                new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.DIFFERENTIAL_ARM_OUT),
                                new SetArmPitchCommand(differentialArmSubsystem, DifferentialArmPitch.STORAGE))
                        .onlyIfLog(
                                () ->
                                        differentialArmSubsystem.getPitchSetpoint()
                                                        != DifferentialArmPitch.STORAGE.getRotation()
                                                && differentialArmSubsystem.pitchAtSetpoint()),
                new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.ZERO),
                new SetAlgeePivotCommand(algeePivotSubsystem, AlgeePivotAngle.OUTFORCORAL),
                new OuttakeAlgeeCommand(algeeWheelSubsystem, 0.2));
    }
}
