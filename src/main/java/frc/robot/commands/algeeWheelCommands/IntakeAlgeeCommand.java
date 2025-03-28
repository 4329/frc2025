package frc.robot.commands.algeeWheelCommands;

import frc.robot.subsystems.AlgeeWheelSubsystem;
import frc.robot.utilities.loggedComands.LoggedCommandComposer;

public class IntakeAlgeeCommand extends LoggedCommandComposer {

    private AlgeeWheelSubsystem algeeWheelSubsystem;

    public IntakeAlgeeCommand(AlgeeWheelSubsystem algeeWheelSubsystem) {
        this.algeeWheelSubsystem = algeeWheelSubsystem;
    }

    @Override
    public void initialize() {
        algeeWheelSubsystem.run(1);
    }

    @Override
    public void end(boolean interrupted) {
        algeeWheelSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
