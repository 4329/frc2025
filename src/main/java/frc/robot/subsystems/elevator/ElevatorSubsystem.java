package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;

public interface ElevatorSubsystem extends Subsystem, LoggedSubsystem {

    public static final double ELEVATOR_DIST = Units.inchesToMeters(13.75);
    public static final double ELEVATOR_START = 0.8525;

    public static final double ELEVATOR_SCORE = 7;

    final double MIN = -50;
    final double MAX = 267;

    public enum ElevatorPosition {
        L2(55.26),
        L2Score(10),

        L3(125.46),
        L3Score(57.81),

        L4(196),
        L4AGAIN(L4.pos),

        ALGEE_LOW(30.26),
        ALGEE_HIGH(103.41),

        NET(MAX - 10),

        ZERO(0),
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
