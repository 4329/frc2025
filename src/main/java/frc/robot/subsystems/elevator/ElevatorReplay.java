package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorReplay implements ElevatorSubsystem {

    @Override
    public LoggableInputs log() {
        return null;
    }

    @Override
    public void setSetpoint(ElevatorPosition setpoint) {}

    @Override
    public void runElevator(double speed) {}

    @Override
    public boolean atSetpoint() {
        return true;
    }

    @Override
    public void enableLimits(boolean enabled) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'enableLimits'");
    }
}
