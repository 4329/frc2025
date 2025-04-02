package frc.robot.subsystems.swerve.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

    @AutoLog
    public static class GyroIOLog {
        public Rotation2d gyro = Rotation2d.kZero;
        public Rotation2d offset = Rotation2d.kZero;
    }

    public default void resetOffset(Rotation2d offset) {}

    public default void updateInputs(GyroIOLogAutoLogged inputs) {}
}
