package frc.robot.commands.algeeWheelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgeeWheelSubsystem;
import frc.robot.subsystems.light.LEDState;

public class IntakeAlgeeCommand extends Command {

    private AlgeeWheelSubsystem algeeWheelSubsystem;
    private double speed;

    public IntakeAlgeeCommand(AlgeeWheelSubsystem algeeWheelSubsystem, double speed) {
        this.algeeWheelSubsystem = algeeWheelSubsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        algeeWheelSubsystem.run(speed);
    }

	@Override
	public void execute() {
		if (algeeWheelSubsystem.getAlgeed()) cancel();
	}

    @Override
    public void end(boolean interrupted) {
		if (interrupted) LEDState.algeeWheelHolding = true;

        algeeWheelSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
