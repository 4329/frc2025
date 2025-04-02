package frc.robot.subsystems.swerve.drivetrain;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIONavX implements GyroIO {
    private AHRS ahrs = new AHRS(NavXComType.kMXP_SPI);

    private Rotation2d offset = Rotation2d.kZero;

    public GyroIONavX() {
        ahrs.reset();
    }

    private Rotation2d getSelf() {
        return ahrs.getRotation2d().rotateBy(Rotation2d.kZero);
    }

    @Override
    public void updateInputs(GyroIOLogAutoLogged inputs) {
        inputs.gyro = getSelf();
        inputs.offset = offset;
    }

    @Override
    public void resetOffset(Rotation2d offset) {
        this.offset = getSelf().unaryMinus().plus(offset);
    }
}
