package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.algeePivotCommands.SetAlgeePivotCommand;
import frc.robot.commands.differentialArmCommands.SetArmPitchCommand;
import frc.robot.commands.differentialArmCommands.SetArmPositionCommand;
import frc.robot.commands.elevatorCommands.SetElevatorCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem.DifferentialArmPitch;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.utilities.loggedComands.LoggedSequentialCommandGroup;


public class MustDoInAutoCommand extends LoggedSequentialCommandGroup {

    public MustDoInAutoCommand(
            AlgeePivotSubsystem algeePivotSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            DifferentialArmSubsystem differentialArmSubsystem
            ) {

        addCommands(
        new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.INTAKE),
        new SetArmPitchCommand(differentialArmSubsystem,DifferentialArmPitch.NINETY),
        new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.ALGEE_CLAW_OUT),
        new SetAlgeePivotCommand(algeePivotSubsystem, AlgeePivotAngle.OUT)
        );
}
}
