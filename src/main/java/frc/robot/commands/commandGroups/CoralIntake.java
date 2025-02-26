package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.differentialArmCommands.SetArmPitchCommand;
import frc.robot.commands.elevatorCommands.SetElevatorCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem.DifferentialArmPitch;
import frc.robot.utilities.NotFinalSequentialCommandGroup;

public class CoralIntake extends NotFinalSequentialCommandGroup {

    public CoralIntake(
            ElevatorSubsystem elevatorSubsystem, DifferentialArmSubsystem differentialArmSubsystem) {

        addCommands(
                new ParallelCommandGroup(
                        new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.INTAKE),
                        new SetArmPitchCommand(differentialArmSubsystem, DifferentialArmPitch.STORAGE)),
                new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.ZERO));
    }
}
