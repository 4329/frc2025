package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.model.ElevatorLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.SparkFactory;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorSubsystem extends SubsystemBase implements LoggedSubsystem {
    private final double ELEVATOR_SPEED = .5;

    private final double MIN = -22.589;
    private final double MAX = 58.91;

    private final double MAX_INPUT_CONSTANT_K = 0.4329;

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

    SparkMax motor1;
    SparkMax motor2;

    RelativeEncoder motor1Encoder;
    PIDController elevatorPID;

    private final ElevatorLogAutoLogged elevatorLogAutoLogged;

    public ElevatorSubsystem() {
        motor1 = SparkFactory.createSparkMax(1000);
        motor2 = SparkFactory.createSparkMax(1200);

        SparkBaseConfig configgled =
                new SparkMaxConfig()
                        .apply(
                                new SoftLimitConfig()
                                        .forwardSoftLimit(MAX)
                                        .forwardSoftLimitEnabled(true)
                                        .reverseSoftLimit(MIN)
                                        .reverseSoftLimitEnabled(true));

        final double gear1 = 11;
        final double gear2 = 54;
        final double gear3 = 28;
        final double belt = 5;
        configgled.encoder.positionConversionFactor(
                (gear1 / gear2) * (gear2 / gear3) * (gear3 / belt)); // approximation

        motor1.configure(configgled, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motor2.configure(
                new SparkMaxConfig().follow(motor1, true),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

        motor1Encoder = motor1.getEncoder();
        elevatorPID = new PIDController(0.1, 0, 0);

        elevatorLogAutoLogged = new ElevatorLogAutoLogged();
    }

    private void setSetpoint(double setpoint) {

        elevatorPID.setSetpoint(MathUtils.clamp(MIN, MAX, setpoint));
    }

    public void setSetpoint(ElevatorPosition setpoint) {
        setSetpoint(setpoint.pos);
    }

    public void runElevator(double speed) {
        setSetpoint(elevatorPID.getSetpoint() + speed * ELEVATOR_SPEED);
    }

    public boolean atSetpoint() {
        return elevatorPID.atSetpoint();
    }

    @Override
    public void periodic() {
        motor1.set(
                MathUtils.clamp(
                        -MAX_INPUT_CONSTANT_K,
                        MAX_INPUT_CONSTANT_K,
                        elevatorPID.calculate(motor1Encoder.getPosition())));
    }

    @Override
    public LoggableInputs log() {
        elevatorLogAutoLogged.setpoint = elevatorPID.getSetpoint();
        return elevatorLogAutoLogged;
    }
}
