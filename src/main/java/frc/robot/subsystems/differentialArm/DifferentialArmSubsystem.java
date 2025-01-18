package frc.robot.subsystems.differentialArm;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;

public interface DifferentialArmSubsystem extends Subsystem, LoggedSubsystem {

    public enum DifferentialArmPitch {
        STORAGE(0), THIRTY(Math.PI / 3), NINETY(2 * Math.PI / 5);

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
}