package frc.robot.subsystems.swerve.drivetrain;

import org.littletonrobotics.junction.Logger;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gyro extends SubsystemBase implements GyroIO {

    GyroIO io;
    GyroIOLogAutoLogged input;

    public Gyro() {
        io = switch (Constants.robotMode) {
            case REPLAY -> {
                yield new GyroIO() {};
            }
            default -> new GyroIONavX();
        };

        input = new GyroIOLogAutoLogged();
    }

    public Rotation2d getRaw() {
        return input.gyro;
    }

    public Rotation2d get() {
        return input.gyro.minus(input.offset);
    }

    public void resetOffset(Rotation2d offset) {
        io.resetOffset(offset);
    }

    @Override
    public void periodic() {
        io.updateInputs(input);
        Logger.processInputs("Gyro", input);
    }
}
