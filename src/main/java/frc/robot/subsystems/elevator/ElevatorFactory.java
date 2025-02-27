package frc.robot.subsystems.elevator;

import frc.robot.Constants;
import java.util.function.Supplier;

public class ElevatorFactory {
    public static ElevatorSubsystem createElevatorSubsystem(Supplier<Double> armAngle) {
        return switch (Constants.robotMode) {
            case REAL -> new ElevatorImpl(armAngle);
            case SIM -> new ElevatorSim(armAngle);
            default -> new ElevatorReplay();
        };
    }
}
