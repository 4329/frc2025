package frc.robot.commands.limitless;

import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.utilities.loggedComands.LoggedCommandComposer;

public class LimitlessRunDifferentialArmCommand extends LoggedCommandComposer {

	DifferentialArmSubsystem differentialArmSubsystem;
	double sign;

	public LimitlessRunDifferentialArmCommand(DifferentialArmSubsystem differentialArmSubsystem, double sign) {
		this.differentialArmSubsystem = differentialArmSubsystem;
		this.sign = sign;
	}

	@Override
	public void initialize() {
		differentialArmSubsystem.enableLimits(false);
	}

	@Override
	public void execute() {
		differentialArmSubsystem.runPitch(sign);
	}

	@Override
	public void end(boolean interrupted) {
		differentialArmSubsystem.enableLimits(true);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
