package frc.robot.subsystems.swerve.module;

import frc.robot.Constants;
import frc.robot.utilities.HoorayConfig;

public class SwerveModuleFactory {
    public static SwerveModule makeSwerve(
            int driveMotorChannel,
            int turningMotorChannel,
            int turningEncoderChannel,
            double angularOffset,
            double[] tuningVals) {
        return HoorayConfig.gimmeConfig().getHasDrivetrain()
                ? switch (Constants.robotMode) {
                    case REAL -> new SwerveModuleImpl(
                            driveMotorChannel,
                            turningMotorChannel,
                            turningEncoderChannel,
                            angularOffset,
                            tuningVals);
                    case SIM -> new SwerveModuleSim(
                            driveMotorChannel, turningMotorChannel, turningEncoderChannel, tuningVals);
                    default -> new SwerveModulePlayback();
                }
                : new SwerveModulePlayback();
    }
}
