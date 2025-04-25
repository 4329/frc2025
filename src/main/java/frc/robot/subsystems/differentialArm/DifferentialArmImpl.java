package frc.robot.subsystems.differentialArm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.model.DifferentialArmLogAutoLogged;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.SparkFactory;
import frc.robot.utilities.shufflebored.*;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class DifferentialArmImpl extends SubsystemBase implements DifferentialArmSubsystem {
    private final double PITCH_SPEED = .05;

    private final double MIN_PITCH = 0;
    private final double MAX_PITCH = 2.95;

    private final DifferentialArmLogAutoLogged differentialArmLogAutoLogged;

    SparkMax motor1;
    SparkMax motor2;

    RelativeEncoder encoder1;

    ProfiledPIDController pitchPID;
    // ArmFeedforward feedforward;

    private double pidCalc;
    private double ffCalc;

    GenericEntry a = Shuffleboard.getTab("Asdf").add("kg", 0).getEntry();

    public DifferentialArmImpl() {
        motor1 = SparkFactory.createSparkMax(Constants.SparkIDs.differential1);
        motor1.configure(
                configureMotor(), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        motor2 = SparkFactory.createSparkMax(Constants.SparkIDs.differential2);
        motor2.configure(
                configureMotor().follow(motor1, true),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

        encoder1 = motor1.getEncoder();

        pitchPID =
                new ShuffledTrapezoidController(0.2, 1, 0.0025, new TrapezoidProfile.Constraints(25, 18));
        pitchPID.setIZone(0.3);
        pitchPID.setTolerance(0.1);
        pitchPID.setGoal(0);

        Shuffleboard.getTab("Asdf").add("diff", pitchPID);
        // feedforward = new ArmFeedforward(0, 0.04, 0);

        differentialArmLogAutoLogged = new DifferentialArmLogAutoLogged();
    }

    private SparkBaseConfig configureMotor() {
        SparkBaseConfig config1 = new SparkMaxConfig().smartCurrentLimit(40);
        config1.encoder.positionConversionFactor((11.0 / 72.0) * (18.0 / 28.0) * (Math.PI * 2.0));
        return config1;
    }

    @Override
    public void setPitchTarget(DifferentialArmPitch pitchTarget) {
        setPitchTarget(pitchTarget.rotation);
    }

    @Override
    public void setPitchTarget(double pitchTarget) {
        pitchPID.setGoal(MathUtils.clamp(MIN_PITCH, MAX_PITCH, pitchTarget));
    }

    @Override
    public void runPitch(double sign) {
        setPitchTarget(pitchPID.getGoal().position + PITCH_SPEED * sign);
    }

    @Override
    public double getPitch() {
        return encoder1.getPosition();
    }

    @Override
    public double getPitchSetpoint() {
        return pitchPID.getGoal().position;
    }

    @Override
    public boolean pitchAtSetpoint() {
        return pitchPID.atGoal();
    }

    Timer timer = new Timer();

    @Override
    public void periodic() {
        pidCalc = pitchPID.calculate(getPitch());
        // if (!pitchAtSetpoint()) timer.restart();
        // if (timer.hasElapsed(0.5)) pitchPID.reset(getPitchSetpoint());
        // ffCalc = feedforward.calculate(getPitch() - Math.PI / 2.0, encoder1.getVelocity());
        motor1.set(pidCalc);

        // feedforward = new ArmFeedforward(0, a.getDouble(0), 0);
    }

    @Override
    public void voltageDrive(Voltage voltage) {
        motor1.setVoltage(voltage);
    }

    @Override
    public void logMotors(SysIdRoutineLog log) {}

    @Override
    public LoggableInputs log() {
        differentialArmLogAutoLogged.pitch = getPitch();
        differentialArmLogAutoLogged.pitchTarget = pitchPID.getGoal().position;
        differentialArmLogAutoLogged.atSetpoint = pitchAtSetpoint();

        differentialArmLogAutoLogged.pidCalc = pidCalc;
        // differentialArmLogAutoLogged.ffCalc = ffCalc;

        return differentialArmLogAutoLogged;
    }
}
