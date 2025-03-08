package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem.DifferentialArmPitch;
import frc.robot.utilities.ButtonRingController;

public class PositionCoralCommand extends Command {

	ElevatorSubsystem elevatorSubsystem;
	DifferentialArmSubsystem differentialArmSubsystem;
	ButtonRingController buttonRingController;

	public PositionCoralCommand(ElevatorSubsystem elevatorSubsystem, DifferentialArmSubsystem differentialArmSubsystem, ButtonRingController buttonRingController) {
		this.elevatorSubsystem = elevatorSubsystem;
		this.differentialArmSubsystem = differentialArmSubsystem;
		this.buttonRingController = buttonRingController;
	}

	@Override
	public void initialize() {
		elevatorSubsystem.setSetpoint(switch (buttonRingController.getLevel()) {
			case 2 -> ElevatorPosition.L2;
			case 3 -> ElevatorPosition.L3;
			case 4 -> ElevatorPosition.L4;
			default -> ElevatorPosition.L2;
		});
	}

	@Override
	public boolean isFinished() {
		return elevatorSubsystem.atSetpoint() && differentialArmSubsystem.pitchAtSetpoint();
	}
}
