package frc.robot.commands.algeeWheelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgeeWheelSubsystem;

public class ToggleAlgeeWheelCommand extends Command {

    private AlgeeWheelSubsystem algeeWheelSubsystem;
    private double speed;

    public ToggleAlgeeWheelCommand(AlgeeWheelSubsystem algeeWheelSubsystem, double speed) {
        this.algeeWheelSubsystem = algeeWheelSubsystem;
        this.speed = speed;

        addRequirements(algeeWheelSubsystem);
    }

    @Override
    public void initialize() {
        algeeWheelSubsystem.run(speed);
    }

    @Override
    public void end(boolean interrupted) {
        algeeWheelSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return algeeWheelSubsystem.getCurrent() > 100;
    }
}
