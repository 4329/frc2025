package frc.robot.commands.limitless;

import frc.robot.utilities.loggedComands.LoggedCommandComposer;

import java.util.function.Supplier;

import frc.robot.subsystems.elevator.*;

public class LimitlessRunElevatorCommand extends LoggedCommandComposer {

    ElevatorSubsystem elevatorSubsystem;
    Supplier<Double> sign;

    public LimitlessRunElevatorCommand(ElevatorSubsystem elevatorSubsystem, Supplier<Double> sign) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.sign = sign;
    }

    @Override
    public void initialize() {
        elevatorSubsystem.enableLimits(false);
    }

    @Override
    public void execute() {
        elevatorSubsystem.runElevator(sign.get());
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.enableLimits(true);
    }

	@Override
	public boolean isFinished() {
		return false;
	}
}
