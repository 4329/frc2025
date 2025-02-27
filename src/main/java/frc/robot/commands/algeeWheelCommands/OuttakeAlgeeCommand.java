package frc.robot.commands.algeeWheelCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgeeWheelSubsystem;
import frc.robot.subsystems.light.LEDState;

public class OuttakeAlgeeCommand extends Command {

    private AlgeeWheelSubsystem algeeWheelSubsystem;
    private double speed;
    private Timer timer;

    public OuttakeAlgeeCommand(AlgeeWheelSubsystem algeeWheelSubsystem) {
        this.algeeWheelSubsystem = algeeWheelSubsystem;
        this.speed = 1;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.start();
        algeeWheelSubsystem.run(speed);
    }

    @Override
    public void end(boolean interrupted) {
        algeeWheelSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(0.3)) {
			LEDState.algeeWheelHolding = false;
            return true;
        } else {
            return false;
        }
    }
}
