package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.differentialArmCommands.SetArmPitchCommand;
import frc.robot.commands.elevatorCommands.SetElevatorCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;

public class ResetAllCommand extends ParallelCommandGroup {

    public ResetAllCommand(
            DifferentialArmSubsystem differentialArmSubsystem, ElevatorSubsystem elevatorSubsystem) {

        addCommands(
                new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.ZERO),
                new SetArmPitchCommand(
                        differentialArmSubsystem, DifferentialArmSubsystem.DifferentialArmPitch.STORAGE));
    }
}
