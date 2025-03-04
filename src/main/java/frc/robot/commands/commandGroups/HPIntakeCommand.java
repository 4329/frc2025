package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.differentialArmCommands.SetArmPitchCommand;
import frc.robot.commands.elevatorCommands.SetElevatorCommand;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem.DifferentialArmPitch;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
public class HPIntakeCommand extends ParallelCommandGroup {

    public HPIntakeCommand(
            ElevatorSubsystem elevatorSubsystem,
            DifferentialArmSubsystem differentialArmSubsystem
            ) {

        addCommands(
                new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.INTAKE),
                new SetArmPitchCommand(differentialArmSubsystem, DifferentialArmPitch.STORAGE));
    }
}
