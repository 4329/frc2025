package frc.robot.subsystems.swerve.drivetrain;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.SparkIDs;

public class GyroIOPigeon implements GyroIO {

    private Pigeon2 pigeon = new Pigeon2(SparkIDs.pigeon);
    private Rotation2d offset;

    public GyroIOPigeon() {
        pigeon
                .getConfigurator()
                .apply(
                        new MountPoseConfigs().withMountPosePitch(0).withMountPoseRoll(0).withMountPoseYaw(0));
    }

    private Rotation2d getSelf() {
        return pigeon.getRotation2d().rotateBy(Rotation2d.kZero);
    }

    @Override
    public void resetOffset(Rotation2d offset) {
        // this.offset = getSelf().unaryMinus().plus(offset);
        this.offset = getSelf().plus(offset);
    }

    @Override
    public void updateInputs(GyroIOLogAutoLogged inputs) {
        inputs.gyro = getSelf();
        inputs.offset = offset;
    }
}
