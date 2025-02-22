package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;

public interface ElevatorSubsystem extends Subsystem, LoggedSubsystem {

    public enum ElevatorPosition {
        L2(-1.778),
        L3(13.972),
        L4(38.347),
        MAX_HEIGHT(58.91),
        ;

        double pos;

        ElevatorPosition(double pos) {
            this.pos = pos;
        }
    }

    public void setSetpoint(ElevatorPosition setpoint);

    public void runElevator(double speed);
}
