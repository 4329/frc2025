package frc.robot.commands.commandGroups;

import frc.robot.commands.algeePivotCommands.SetAlgeePivotCommand;
import frc.robot.commands.algeeWheelCommands.IntakeAlgeeCommand;
import frc.robot.commands.algeeWheelCommands.OuttakeAlgeeCommand;
import frc.robot.commands.elevatorCommands.SetElevatorCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import frc.robot.subsystems.AlgeeWheelSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.utilities.ButtonRingController;
import frc.robot.utilities.UnInstantCommand;
import frc.robot.utilities.loggedComands.LoggedSequentialCommandGroup;

public class AlgeeIntakeLameCommand extends LoggedSequentialCommandGroup {

	ButtonRingController buttonRingController;

    public AlgeeIntakeLameCommand(
            ElevatorSubsystem elevatorSubsystem,
            AlgeePivotSubsystem algeePivotSubsystem,
            AlgeeWheelSubsystem algeeWheelSubsystem,
			ButtonRingController buttonRingController) {

		this.buttonRingController = buttonRingController;

        addCommands(
				new UnInstantCommand(
					"ElevatorAlgeeButtonRing",
					() -> elevatorSubsystem.setSetpoint(switch (buttonRingController.getLevel()) {
						case 2 -> ElevatorPosition.ALGEE_LOW;
						case 3 -> ElevatorPosition.ALGEE_HIGH;
						default -> ElevatorPosition.ALGEE_LOW;
					})).whileLog(() -> !elevatorSubsystem.atSetpoint()),
                new SetAlgeePivotCommand(algeePivotSubsystem, AlgeePivotAngle.OUT),
                new IntakeAlgeeCommand(algeeWheelSubsystem));
    }

	@Override
	public void execute() {
		if (buttonRingController.getLevel() == 2 || buttonRingController.getLevel() == 3) super.execute();
	}
}
