package frc.robot.commands.algeeWheelCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgeeWheelSubsystem;
import frc.robot.subsystems.light.LEDState;

public class IntakeAlgeeCommand extends Command {

    private AlgeeWheelSubsystem algeeWheelSubsystem;
    private double speed;
    private final Timer timer = new Timer();
    private final double WAIT_TIME = 0.2;

    public IntakeAlgeeCommand(AlgeeWheelSubsystem algeeWheelSubsystem, double speed) {
        this.algeeWheelSubsystem = algeeWheelSubsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        timer.restart();
        algeeWheelSubsystem.run(speed);
    }

    @Override
    public void execute() {
        System.out.println(timer.get());
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) LEDState.algeeWheelHolding = true;

        algeeWheelSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.get() > WAIT_TIME && algeeWheelSubsystem.getAlgeed();
    }
}
