package frc.robot.commands.algeeWheelCommands;

import frc.robot.subsystems.AlgeeWheelSubsystem;
import frc.robot.subsystems.light.LEDState;
import frc.robot.utilities.loggedComands.LoggedCommandComposer;

public class OuttakeAlgeeCommand extends LoggedCommandComposer {

    private AlgeeWheelSubsystem algeeWheelSubsystem;
    private double speed;

    public OuttakeAlgeeCommand(AlgeeWheelSubsystem algeeWheelSubsystem) {
        this(algeeWheelSubsystem, 1);
    }

    public OuttakeAlgeeCommand(AlgeeWheelSubsystem algeeWheelSubsystem, double speed) {
        this.algeeWheelSubsystem = algeeWheelSubsystem;
        this.speed = -speed;
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
        return false;
    }
}
