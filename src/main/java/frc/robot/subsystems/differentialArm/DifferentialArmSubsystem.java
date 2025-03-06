package frc.robot.subsystems.differentialArm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;

public interface DifferentialArmSubsystem extends Subsystem, LoggedSubsystem {

    public static final double ARM_LENGTH_CORAL_CENTER = Units.inchesToMeters(17.151904);
    public static final double ARM_LENGTH_CLAW_END = Units.inchesToMeters(18.151904);

    public enum DifferentialArmPitch {
        STORAGE(0),
        THIRTY(Math.PI / 3),
        NINETY(2 * Math.PI / 5),
        ONETHIRTYFIVE(3 * Math.PI / 4);

        double rotation;

        DifferentialArmPitch(double rotation) {
            this.rotation = rotation;
        }
    }

    void setPitchTarget(DifferentialArmPitch pitchTarget);

    void setPitchTarget(double pitchTarget);

    void setRollTarget(double rollTarget);

    void runPitch(double sign);

    void runRoll(double sign);

    double getPitch();

    double getRoll();

    boolean pitchAtSetpoint();

    boolean rollAtSetpoint();

    @Override
    public default String getNameLog() {
        return "DifferentialArmSubsystem";
    }
}
