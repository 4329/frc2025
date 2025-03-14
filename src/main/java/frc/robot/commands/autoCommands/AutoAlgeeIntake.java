package frc.robot.commands.autoCommands;

import frc.robot.commands.algeePivotCommands.SetAlgeePivotCommand;
import frc.robot.commands.algeeWheelCommands.IntakeAlgeeCommand;
import frc.robot.commands.driveCommands.CenterOnAlgeeCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import frc.robot.subsystems.AlgeeWheelSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.ButtonRingController;
import frc.robot.utilities.CenterDistance;
import frc.robot.utilities.UnInstantCommand;
import frc.robot.utilities.loggedComands.LoggedParallelCommandGroup;
import frc.robot.utilities.loggedComands.LoggedSequentialCommandGroup;

public class AutoAlgeeIntake extends LoggedSequentialCommandGroup {

	private ButtonRingController buttonRingController;
	AlgeeWheelSubsystem algeeWheelSubsystem;

	public AutoAlgeeIntake(
			Drivetrain drivetrain,
			ElevatorSubsystem elevatorSubsystem,
			AlgeeWheelSubsystem algeeWheelSubsystem,
			AlgeePivotSubsystem algeePivotSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem,
			ElevatorPosition elevatorPosition) {

			this.algeeWheelSubsystem = algeeWheelSubsystem;

		addCommands(
				new SetAlgeePivotCommand(algeePivotSubsystem, AlgeePivotAngle.OUT),
				new UnInstantCommand("intake algee", () -> elevatorSubsystem.setSetpoint(
                ElevatorPosition.L3.equals(elevatorPosition)
                        ? ElevatorPosition.ALGEE_HIGH
                        : ElevatorPosition.ALGEE_LOW)));
	}
}
