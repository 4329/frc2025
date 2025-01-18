package frc.robot.subsystems.differentialArmSubsystem;

import frc.robot.Constants;

public class DifferentialArmFactory {
    public static DifferentialArmSubsystem createDifferentialArmSubsystem() {
        return switch(Constants.robotMode) {
            case REAL -> new DifferentialArmImpl();
            case SIM -> new DifferentialArmSim();
            default ->  new DifferentialArmPlayback();
        };
    }
}
