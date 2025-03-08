package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.algeePivotCommands.SetAlgeePivotCommand;
import frc.robot.commands.differentialArmCommands.SetArmPitchCommand;
import frc.robot.commands.elevatorCommands.SetElevatorCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.utilities.loggedComands.LoggedParallelCommandGroup;

public class ResetAllCommand extends LoggedParallelCommandGroup {

    public ResetAllCommand(
            DifferentialArmSubsystem differentialArmSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            AlgeePivotSubsystem algeePivotSubsystem) {

        addCommands(
				new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.DIFFERENTIAL_ARM_OUT),
				new SetArmPitchCommand(
					differentialArmSubsystem, DifferentialArmSubsystem.DifferentialArmPitch.STORAGE)
				);
    }
}
