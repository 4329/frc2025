package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModulePlayback implements SwerveModule {

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState();
    }

    @Override
    public SwerveModuleState getStateNoOffset() {
        return new SwerveModuleState();
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {}

    @Override
    public void stop() {}

    @Override
    public double getTurnEncoder() {
        return 0.0;
    }

    @Override
    public void brakeModeModule() {}

    @Override
    public void coastModeModule() {}

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition();
    }
}
