package frc.robot.subsystems.swerve.drivetrain;

import frc.robot.Constants;

public class DrivetrainFactory {
    public static Drivetrain makeDrivetrain() {
        return switch (Constants.robotMode) {
			case REAL -> new DrivetrainImpl();
            case SIM -> new DrivetrainSim();
            case REPLAY -> new DrivetrainReplay();
        };
    }
}
