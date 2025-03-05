package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.model.ElevatorLogAutoLogged;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.SparkFactory;
import java.util.function.Supplier;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorImpl extends SubsystemBase implements ElevatorSubsystem {
    private GenericEntry a = Shuffleboard.getTab("Asdf").add("0", 2).getEntry();
    private GenericEntry speed = Shuffleboard.getTab("Asdf").add("speed", 160).getEntry();
    private GenericEntry accel = Shuffleboard.getTab("Asdf").add("accel", 240).getEntry();

    private double ELEVATOR_SPEED = 2;

    final double MIN = -50;
    final double MAX = 267;

    private final double MAX_OUTPUT_CONSTANT_K = 1;

    SparkMax motor1;
    SparkMax motor2;

    RelativeEncoder motor1Encoder;
    ProfiledPIDController elevatorPID;

    private final ElevatorLogAutoLogged elevatorLogAutoLogged;

    private final Supplier<Double> armAngle;

    private TrapezoidProfile.Constraints profile = new TrapezoidProfile.Constraints(160, 240);

    public ElevatorImpl(Supplier<Double> armAngle) {
        motor1 = SparkFactory.createSparkMax(Constants.SparkIDs.elevator1);
        motor2 = SparkFactory.createSparkMax(Constants.SparkIDs.elevator2);

        SparkBaseConfig configgled = new SparkMaxConfig().inverted(true);
        // .apply(
        // new SoftLimitConfig()
        //         .forwardSoftLimit(MAX)
        //         .forwardSoftLimitEnabled(true)
        //         .reverseSoftLimit(MIN)
        //         .reverseSoftLimitEnabled(true));

        // final double pulleyTeeth = 28;
        // final double belt = 5.5 / Units.inchesToMeters(1);
        // final double gearRatio = 1 / 25;
        // configgled.encoder.positionConversionFactor(pulleyTeeth / belt * gearRatio);

        motor1.configure(configgled, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motor2.configure(
                new SparkMaxConfig().follow(motor1, true),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

        motor1Encoder = motor1.getEncoder();
        elevatorPID = new ProfiledPIDController(0.09, 0, 0, profile);
        Shuffleboard.getTab("Asdf").add("a", elevatorPID);

        elevatorLogAutoLogged = new ElevatorLogAutoLogged();
        this.armAngle = armAngle;
    }

    private void setSetpoint(double setpoint) {
        double armLengthY =
                Math.abs(DifferentialArmSubsystem.ARM_LENGTH_CLAW_END / Math.cos(armAngle.get()));
        elevatorPID.setGoal(MathUtils.clamp(MIN + armLengthY, MAX, setpoint));
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
        return elevatorPID.atSetpoint();
    }

    @Override
    public void periodic() {
        elevatorPID.setConstraints(
                new TrapezoidProfile.Constraints(speed.getDouble(0), accel.getDouble(0)));
        ELEVATOR_SPEED = a.getDouble(0);

        motor1.set(
                MathUtils.clamp(
                        -MAX_OUTPUT_CONSTANT_K,
                        MAX_OUTPUT_CONSTANT_K,
                        elevatorPID.calculate(motor1Encoder.getPosition())));
    }

    @Override
    public LoggableInputs log() {
        elevatorLogAutoLogged.setpoint = elevatorPID.getGoal().position;
        elevatorLogAutoLogged.position = motor1Encoder.getPosition();
        return elevatorLogAutoLogged;
    }
}
