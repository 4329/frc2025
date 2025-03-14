package frc.robot.subsystems.differentialArm;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class DifferentialArmPlayback extends SubsystemBase implements DifferentialArmSubsystem {

    @Override
    public LoggableInputs log() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'log'");
    }

    @Override
    public void setPitchTarget(DifferentialArmPitch pitchTarget) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPitchTarget'");
    }

    @Override
    public void setPitchTarget(double pitchTarget) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setRollTarget'");
    }

    @Override
    public double getPitch() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPitch'");
    }

    @Override
    public double getPitchSetpoint() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPitchSetpoint'");
    }

    @Override
    public boolean pitchAtSetpoint() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'pitchAtSetpoint'");
    }

    @Override
    public void runPitch(double sign) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'runPitch'");
    }

    @Override
    public void voltageDrive(Voltage voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'voltageDrive'");
    }

    @Override
    public void logMotors(SysIdRoutineLog log) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'logMotors'");
    }
}
