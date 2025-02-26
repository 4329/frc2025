package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.model.ElevatorLogAutoLogged;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.SparkFactory;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorImpl extends SubsystemBase implements ElevatorSubsystem {
    private final double ELEVATOR_SPEED = .01;

    final double MIN = 0.2785 - ELEVATOR_START;
    final double MAX = 2.3488 - ELEVATOR_START;

    private final double MAX_INPUT_CONSTANT_K = 0.4329;

    SparkMax motor1;
    SparkMax motor2;

    RelativeEncoder motor1Encoder;
    PIDController elevatorPID;

    private final ElevatorLogAutoLogged elevatorLogAutoLogged;

    public ElevatorImpl() {
        motor1 = SparkFactory.createSparkMax(Constants.SparkIDs.elevator1);
        motor2 = SparkFactory.createSparkMax(Constants.SparkIDs.elevator2);

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
        final double belt = .127;
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

    @Override
    public void setSetpoint(ElevatorSubsystem.ElevatorPosition setpoint) {
        setSetpoint(setpoint.pos);
    }

    @Override
    public void runElevator(double speed) {
        setSetpoint(elevatorPID.getSetpoint() + speed * ELEVATOR_SPEED);
    }

    @Override
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
