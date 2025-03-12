package frc.robot.commands.algeeWheelCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgeeWheelSubsystem;
import frc.robot.subsystems.light.LEDState;

public class IntakeAlgeeCommand extends Command {

    private AlgeeWheelSubsystem algeeWheelSubsystem;
    private final Timer timer;
    private final double WAIT_TIME = 0.2;

    public IntakeAlgeeCommand(AlgeeWheelSubsystem algeeWheelSubsystem) {
        this(algeeWheelSubsystem, new Timer());
    }

    public IntakeAlgeeCommand(AlgeeWheelSubsystem algeeWheelSubsystem, Timer timer) {
        this.algeeWheelSubsystem = algeeWheelSubsystem;
        this.timer = timer;
    }

    @Override
    public void initialize() {
        timer.restart();
        algeeWheelSubsystem.run(1);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) LEDState.algeeWheelHolding = true;

        algeeWheelSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(WAIT_TIME) && algeeWheelSubsystem.getAlgeed();
    }
}
