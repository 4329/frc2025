package frc.robot.utilities;

import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;

public enum CenterDistance {
    INITIAL(DifferentialArmSubsystem.ARM_LENGTH_CORAL_CENTER * 2, 0.2, 0.1),
    SCORING(DifferentialArmSubsystem.ARM_LENGTH_CORAL_CENTER, 0.002, 0.01);

    private double zDist;
	private double translationTolerance;
	private double rotationTolerance;

	CenterDistance(double zDist, double translationTolerance, double rotationTolerance) {
		this.zDist = zDist;
		this.translationTolerance = translationTolerance;
		this.rotationTolerance = rotationTolerance;
	}

	public double getzDist() {
		return zDist;
	}

	public double getTranslationTolerance() {
		return translationTolerance;
	}

	public double getRotationTolerance() {
		return rotationTolerance;
	}

}
