package frc.robot.commands.algeeWheelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgeeWheelSubsystem;

public class ToggleAlgeeWheelCommand extends Command {

    private AlgeeWheelSubsystem algeeWheelSubsystem;

    public ToggleAlgeeWheelCommand(AlgeeWheelSubsystem algeeWheelSubsystem) {
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
