package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.model.ElevatorLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;

public interface ElevatorIO {

    public static final double ELEVATOR_DIST = Units.inchesToMeters(13.75);
    public static final double ELEVATOR_START = 0.8525;

    public static final double ELEVATOR_SCORE = 7;

	public default void set(double speed) {}

	public default void updateInputs(ElevatorLogAutoLogged inputs) {}
}
