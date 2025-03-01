package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModulePlayback implements SwerveModule {

	SwerveModuleLogAutoLogged inputs = new SwerveModuleLogAutoLogged();

    @Override
    public SwerveModuleState getState() {
		return inputs.state;
    }

    @Override
    public SwerveModuleState getStateNoOffset() {
		return new SwerveModuleState(inputs.state.speedMetersPerSecond, new Rotation2d(inputs.state.angle.getRadians() - inputs.offset));
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
    }

    @Override
    public void stop() {
    }

    @Override
    public double getTurnEncoder() {
		return inputs.position.angle.getRadians();
    }

    @Override
    public void brakeModeModule() {
    }

    @Override
    public void coastModeModule() {
    }

    @Override
    public SwerveModulePosition getPosition() {
		return inputs.position;
    }

	@Override
	public void updateInputs() {

	}
}
