package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;

public interface ElevatorSubsystem extends Subsystem, LoggedSubsystem {

    public static final double ELEVATOR_DIST = Units.inchesToMeters(13.75);
    public static final double ELEVATOR_START = 0.8525;

    public static final double ELEVATOR_SCORE = 7;

    public enum ElevatorPosition {
        L2(21.7),
        L2Score(24),

        L3(110),
        L3Score(45),

        L4(235),
        L4Score(200),

        ALGEE_LOW(13), // calculate these later WIP
        ALGEE_HIGH(88), // calculate these later WIP

        ZERO(0),
        INTAKE(2), // calculate these later WIPWIP
        PORCESSOR(
                -2), // WIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIP

		DIFFERENTIAL_ARM_OUT(19.7),
		ALGEE_CLAW_OUT(-14.4),
        ;

        double pos;

        ElevatorPosition(double pos) {
            this.pos = pos;
        }
    }

    public void setSetpoint(ElevatorPosition setpoint);

    public void runElevator(double speed);

    public boolean atSetpoint();

    @Override
    public default String getNameLog() {
        return "ElevatorSubsystem";
    }
}
