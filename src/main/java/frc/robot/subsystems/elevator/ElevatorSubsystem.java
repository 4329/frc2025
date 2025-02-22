package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;

public interface ElevatorSubsystem extends Subsystem, LoggedSubsystem {

	public static final double ELEVATOR_START = 0.8525;
    public enum ElevatorPosition {
        L2(0.8073 - ELEVATOR_START),
        L3(1.2073 - ELEVATOR_START),
        L4(1.8265 - ELEVATOR_START),
        MAX_HEIGHT(2.3488 - ELEVATOR_START),
        ;

        double pos;

        ElevatorPosition(double pos) {
            this.pos = pos;
        }
    }

    public void setSetpoint(ElevatorPosition setpoint);

    public void runElevator(double speed);
}
