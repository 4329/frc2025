package frc.robot.subsystems.swerve.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase implements GyroIO {

    GyroIO io;
    GyroIOLogAutoLogged input;

    Gyro() {
        io = new GyroIO() {};

        input = new GyroIOLogAutoLogged();
    }

    public Rotation2d getRaw() {
        return input.gyro;
    }

    public Rotation2d get() {
        return input.gyro.plus(input.offset);
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
