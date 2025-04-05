package frc.robot.subsystems.differentialArm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;

public interface DifferentialArmSubsystem extends Subsystem, LoggedSubsystem {

    public static final double ARM_LENGTH_CORAL_CENTER = Units.inchesToMeters(17.151904);
    public static final double ARM_LENGTH_CLAW_END = Units.inchesToMeters(18.151904);

    public enum DifferentialArmPitch {
        STORAGE(0),
        NINETY(Math.PI / 2),
        NINETY_PLUS((Math.PI / 2) + 0.02),
        NINETY_PLUS_2((Math.PI / 2) + 0.05),
        ONE_THIRTY_FIVE(3 * Math.PI / 4),
        SCORE_LOW((Math.PI / 2) - 0.005),
        FOURTYFIVE(Math.PI / 4);

        double rotation;

        DifferentialArmPitch(double rotation) {
            this.rotation = rotation;
        }

        public double getRotation() {
            return rotation;
        }
        ;
    }

    void setPitchTarget(DifferentialArmPitch pitchTarget);

    void setPitchTarget(double pitchTarget);

    void runPitch(double sign);

    double getPitch();

    double getPitchSetpoint();

    boolean pitchAtSetpoint();

    public void voltageDrive(Voltage voltage);

    public void logMotors(SysIdRoutineLog log);

    @Override
    public default String getNameLog() {
        return "DifferentialArmSubsystem";
    }
}
