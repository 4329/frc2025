package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;

public interface ElevatorSubsystem extends Subsystem, LoggedSubsystem {

    public static final double ELEVATOR_START = 0.8525;

    public enum ElevatorPosition {
        L2(-1.778),
        L3(13.972),
        L4(38.347),
        ALGEE_HIGH(0), // calculate these later WIP
        ALGEE_LOW(0), // calculate these later WIP
        MAX_HEIGHT(58.91),
        ZERO(0),
        INTAKE(2), // calculate these later WIPWIP
        PORCESSOR(
                -2), // WIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIP
        ;

        double pos;

        ElevatorPosition(double pos) {
            this.pos = pos;
        }
    }

    public void setSetpoint(ElevatorPosition setpoint);

    public void runElevator(double speed);

    public boolean atSetpoint();
}
