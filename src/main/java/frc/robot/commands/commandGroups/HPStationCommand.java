package frc.robot.commands.commandGroups;

import frc.robot.commands.differentialArmCommands.SetArmPitchCommand;
import frc.robot.commands.elevatorCommands.SetElevatorCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.utilities.loggedComands.LoggedSequentialCommandGroup;

public class HPStationCommand extends LoggedSequentialCommandGroup {

    public HPStationCommand(
            DifferentialArmSubsystem differentialArmSubsystem,
            ElevatorSubsystem elevatorSubsystem) {

        addCommands(
                new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.DIFFERENTIAL_ARM_OUT),
                new SetArmPitchCommand(
                        differentialArmSubsystem, DifferentialArmSubsystem.DifferentialArmPitch.STORAGE));
    }
}
