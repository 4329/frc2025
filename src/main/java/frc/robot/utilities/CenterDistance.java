package frc.robot.utilities;

import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;

public enum CenterDistance {
    INITIAL(DifferentialArmSubsystem.ARM_LENGTH_CORAL_CENTER * 2),
    SCORING(DifferentialArmSubsystem.ARM_LENGTH_CORAL_CENTER);
    private double zDist;

    public double getzDist() {
        return zDist;
    }

    private CenterDistance(double steve) {
        zDist = steve;
    }
}
