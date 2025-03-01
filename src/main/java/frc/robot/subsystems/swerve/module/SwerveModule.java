package frc.robot.subsystems.swerve.module;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {

	@AutoLog
	public static class SwerveModuleLog {
		public SwerveModuleState state;
		public double offset;
		public SwerveModulePosition position;
	}

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    SwerveModuleState getState();

    SwerveModuleState getStateNoOffset();

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    void setDesiredState(SwerveModuleState desiredState);

    void stop();

    /**
     * Obtains the negative of the turning absolute encoder value as this encoder reads opposite of
     * the module rotation on 2910 MK2 swerve.
     *
     * @return the modified absolute encoder value.
     */
    double getTurnEncoder();

    void brakeModeModule();

    void coastModeModule();

    SwerveModulePosition getPosition();

	void updateInputs();
}
