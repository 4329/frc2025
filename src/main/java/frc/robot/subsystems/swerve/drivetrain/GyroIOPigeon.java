package frc.robot.subsystems.swerve.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.SparkIDs;

public class GyroIOPigeon implements GyroIO {

    private Pigeon2 pigeon = new Pigeon2(SparkIDs.pigeon);
    private Rotation2d offset;

    @Override
    public void resetOffset(Rotation2d offset) {
        this.offset = offset;
    }

    @Override
    public void updateInputs(GyroIOLogAutoLogged inputs) {
        inputs.gyro = pigeon.getRotation2d();
        inputs.offset = offset;
    }

}
