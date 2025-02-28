package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.algeePivotCommands.SetAlgeePivotCommand;
import frc.robot.commands.algeeWheelCommands.OuttakeAlgeeCommand;
import frc.robot.commands.differentialArmCommands.SetArmPitchCommand;
import frc.robot.commands.elevatorCommands.SetElevatorCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import frc.robot.subsystems.AlgeeWheelSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.loggedComands.LoggedParallelCommandGroup;

public class AlgeeSocrePorcessor extends SequentialCommandGroup {

    public AlgeeSocrePorcessor(
            AlgeePivotSubsystem algeePivotSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            DifferentialArmSubsystem differentialArmSubsystem,
            PoseEstimationSubsystem poseEstimationSubsystem,
            Drivetrain drivetrain,
            AlgeeWheelSubsystem algeeWheelSubsystem) {

        addCommands(
                new SetArmPitchCommand(
                        differentialArmSubsystem, DifferentialArmSubsystem.DifferentialArmPitch.NINETY),
                new LoggedParallelCommandGroup(
                        "",
                        new SetAlgeePivotCommand(algeePivotSubsystem, AlgeePivotAngle.OUT),
                        new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.PORCESSOR)),
                new OuttakeAlgeeCommand(algeeWheelSubsystem));
    }
}
