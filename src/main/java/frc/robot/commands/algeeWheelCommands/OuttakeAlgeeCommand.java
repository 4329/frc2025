package frc.robot.commands.algeeWheelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgeeWheelSubsystem;
import frc.robot.subsystems.light.LEDState;

public class OuttakeAlgeeCommand extends Command {

    private AlgeeWheelSubsystem algeeWheelSubsystem;

    public OuttakeAlgeeCommand(AlgeeWheelSubsystem algeeWheelSubsystem) {
        this.algeeWheelSubsystem = algeeWheelSubsystem;
    }

    @Override
    public void initialize() {
        algeeWheelSubsystem.run(-1);
    }

    @Override
    public void end(boolean interrupted) {
        LEDState.algeeWheelHolding = false;
        algeeWheelSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
