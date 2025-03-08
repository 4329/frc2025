package frc.robot.subsystems.elevator;

import frc.robot.Constants;

public class ElevatorFactory {
    public static ElevatorSubsystem createElevatorSubsystem() {
        return switch (Constants.robotMode) {
            case REAL -> new ElevatorImpl();
            case SIM -> new ElevatorSim();
            default -> new ElevatorReplay();
        };
    }
}
