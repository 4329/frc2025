package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.model.ElevatorLogAutoLogged;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.SparkFactory;
import frc.robot.utilities.shufflebored.*;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorImpl extends SubsystemBase implements ElevatorSubsystem {

    private double ELEVATOR_SPEED = 2;


    private final double MAX_OUTPUT_CONSTANT_K = 1;

    SparkMax motor1;
    SparkMax motor2;

    RelativeEncoder motor1Encoder;
    ProfiledPIDController elevatorPID;

    private final ElevatorLogAutoLogged elevatorLogAutoLogged;

    private TrapezoidProfile.Constraints profile = new TrapezoidProfile.Constraints(160, 240);

    public ElevatorImpl() {
        motor1 = SparkFactory.createSparkMax(Constants.SparkIDs.elevator1);
        motor2 = SparkFactory.createSparkMax(Constants.SparkIDs.elevator2);

        SparkBaseConfig configgled =
                new SparkMaxConfig()
                        .inverted(true)
                        .apply(
                                new SoftLimitConfig()
                                        .forwardSoftLimit(MAX)
                                        .forwardSoftLimitEnabled(true)
                                        .reverseSoftLimit(MIN)
                                        .reverseSoftLimitEnabled(true));

        motor1.configure(configgled, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motor2.configure(
                new SparkMaxConfig().follow(motor1, true),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

        motor1Encoder = motor1.getEncoder();
        elevatorPID = new ShuffledTrapezoidController(0.09, 0, 0, profile);
        elevatorPID.setTolerance(1);
        Shuffleboard.getTab("Asdf").add("elevator", elevatorPID);

        elevatorLogAutoLogged = new ElevatorLogAutoLogged();
    }

    private void setSetpoint(double setpoint) {
        // double armLengthY =
        //        Math.abs(DifferentialArmSubsystem.ARM_LENGTH_CLAW_END);
        elevatorPID.setGoal(MathUtils.clamp(MIN, MAX, setpoint));
    }

    @Override
    public void setSetpoint(ElevatorSubsystem.ElevatorPosition setpoint) {
        setSetpoint(setpoint.pos);
    }

    @Override
    public void runElevator(double speed) {
        setSetpoint(elevatorPID.getGoal().position + speed * ELEVATOR_SPEED);
    }

    @Override
    public boolean atSetpoint() {
        return elevatorPID.atGoal();
    }

    @Override
    public void periodic() {

        motor1.set(
                !atSetpoint()
                        ? MathUtils.clamp(
                                -MAX_OUTPUT_CONSTANT_K,
                                MAX_OUTPUT_CONSTANT_K,
                                elevatorPID.calculate(motor1Encoder.getPosition()))
                        : 0);
    }

    @Override
    public LoggableInputs log() {
        elevatorLogAutoLogged.setpoint = elevatorPID.getGoal().position;
        elevatorLogAutoLogged.position = motor1Encoder.getPosition();
        elevatorLogAutoLogged.atSetpoint = atSetpoint();
        return elevatorLogAutoLogged;
    }
}
