package frc.robot.subsystems.differentialArmSubsystem;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;

public interface DifferentialArmSubsystem extends Subsystem, LoggedSubsystem {

    void setPitchTarget(double pitchTarget);
    void setRollTarget(double rollTarget);

    double getPitch();
    double getRoll();

    boolean pitchAtSetpoint();
    boolean rollAtSetpoint();
}