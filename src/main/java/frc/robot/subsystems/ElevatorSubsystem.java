package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.model.ElevatorLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.utilities.SparkFactory;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorSubsystem extends SubsystemBase implements LoggedSubsystem {
    private final double ELEVATOR_SPEED = .01;

    public enum ElevatorPosition {
        LOW(0),
        MIDDLE(50),
        HIGH(100);

        double pos;

        ElevatorPosition(double pos) {
            this.pos = pos;
        }
    }

    SparkMax motor1;
    SparkMax motor2;

    RelativeEncoder motor1Encoder;
    PIDController elevatorPID;

    private final ElevatorLogAutoLogged elevatorLogAutoLogged;

    public ElevatorSubsystem() {
        motor1 = SparkFactory.createSparkMax(11);
        motor2 = SparkFactory.createSparkMax(12);

        motor1.configure(
                new SparkMaxConfig()
                        .apply(
                                new SoftLimitConfig()
                                        .forwardSoftLimit(100)
                                        .forwardSoftLimitEnabled(true)
                                        .reverseSoftLimit(0)
                                        .reverseSoftLimitEnabled(true)),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        motor2.configure(
                new SparkMaxConfig().follow(motor1, true),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

        motor1Encoder = motor1.getEncoder();
        elevatorPID = new PIDController(0.1, 0, 0);

        elevatorLogAutoLogged = new ElevatorLogAutoLogged();
    }

    public void setSetpoint(ElevatorPosition setpoint) {
        elevatorPID.setSetpoint(setpoint.pos);
    }

    public void runElevator(double speed) {
        elevatorPID.setSetpoint(elevatorPID.getSetpoint() + speed * ELEVATOR_SPEED);
    }

    @Override
    public void periodic() {
        motor1.set(elevatorPID.calculate(motor1Encoder.getPosition()));
    }

    @Override
    public LoggableInputs log() {
        elevatorLogAutoLogged.setpoint = elevatorPID.getSetpoint();
        return elevatorLogAutoLogged;
    }
}
