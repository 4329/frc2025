package frc.robot.commands.commandGroups;

import frc.robot.commands.differentialArmCommands.SetArmPitchCommand;
import frc.robot.commands.elevatorCommands.SetElevatorCommand;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem.DifferentialArmPitch;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.utilities.loggedComands.LoggedParallelCommandGroup;
import frc.robot.utilities.loggedComands.LoggedSequentialCommandGroup;

public class CoralIntake extends LoggedSequentialCommandGroup {

    public CoralIntake(
            ElevatorSubsystem elevatorSubsystem, DifferentialArmSubsystem differentialArmSubsystem) {

        addCommands(
                new LoggedParallelCommandGroup(
                        "SetElevatorAndArm",
                        new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.INTAKE),
                        new SetArmPitchCommand(differentialArmSubsystem, DifferentialArmPitch.STORAGE)),
                new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.ZERO));
    }
}
