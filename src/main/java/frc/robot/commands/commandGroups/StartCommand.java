package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.algeePivotCommands.SetAlgeePivotCommand;
import frc.robot.commands.differentialArmCommands.SetArmPitchCommand;
import frc.robot.commands.elevatorCommands.SetElevatorCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem.DifferentialArmPitch;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.utilities.loggedComands.LoggedParallelCommandGroup;
import frc.robot.utilities.loggedComands.LoggedSequentialCommandGroup;
import frc.robot.utilities.loggedComands.LoggedWaitCommand;

public class StartCommand extends LoggedSequentialCommandGroup {

	public StartCommand(
			ElevatorSubsystem elevatorSubsystem,
			DifferentialArmSubsystem differentialArmSubsystem,
			AlgeePivotSubsystem algeePivotSubsystem) {

		addCommands(
				new LoggedParallelCommandGroup(
						"EleAndArm",
						new SetElevatorCommand(elevatorSubsystem,
								ElevatorPosition.DIFFERENTIAL_ARM_OUT),
						new LoggedWaitCommand(0.1)
								.andThenLog(new Command() {
									@Override
									public String getName() {
										return "SkibidiStartArm";
									}

									@Override
									public void initialize() {
										differentialArmSubsystem.setPitchTarget(DifferentialArmPitch.NINETY);
									}

									@Override
									public boolean isFinished() {
										return differentialArmSubsystem.getPitch() >= DifferentialArmPitch.FOURTYFIVE
												.getRotation();
									}
								})),
				new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.ALGEE_CLAW_OUT),
				new SetAlgeePivotCommand(
						algeePivotSubsystem, AlgeePivotSubsystem.AlgeePivotAngle.OUT));
	}
}
