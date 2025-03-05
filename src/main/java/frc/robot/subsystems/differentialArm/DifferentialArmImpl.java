package frc.robot.subsystems.differentialArm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.model.DifferentialArmLogAutoLogged;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.SparkFactory;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class DifferentialArmImpl extends SubsystemBase implements DifferentialArmSubsystem {
    private final double PITCH_SPEED = .05;
    private final double ROLL_SPEED = .05;

    private final double MAX_POWER = 0.5;

    private final double MIN_PITCH = -2.7;
    private final double MAX_PITCH = 0;

    private final double MIN_ROLL = -1000000;
    private final double MAX_ROLL = 10000000;

    private final double FEED_FORWARD1 = 0.05;
    private final double FEED_FORWARD2 = 0.05;

    private final double sharedP = 0.3;
    private final double sharedI = 0.4; // 0.00025
    private final double sharedD = 0.0025;
    private final double sharedIZone = 0.1; // 0.1

    private final DifferentialArmLogAutoLogged differentialArmLogAutoLogged;

    SparkMax motor1;
    SparkMax motor2;

    RelativeEncoder encoder1;
    RelativeEncoder encoder2;

    double pitchTarget;
    double rollTarget;

    ProfiledPIDController pitchPID;
    ProfiledPIDController rollPID;

    public DifferentialArmImpl() {
        motor1 = SparkFactory.createSparkMax(Constants.SparkIDs.differential1);
        motor1.configure(
                configureMotor(), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        motor2 = SparkFactory.createSparkMax(Constants.SparkIDs.differential2);
        motor2.configure(
                configureMotor().inverted(true),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

        encoder1 = motor1.getEncoder();
        encoder2 = motor2.getEncoder();

        pitchPID =
                new ProfiledPIDController(
                        sharedP, sharedI, sharedD, new TrapezoidProfile.Constraints(6, 8));
        pitchPID.setIZone(sharedIZone);
        pitchPID.setGoal(0);

        rollPID =
                new ProfiledPIDController(
                        sharedP, sharedI, sharedD, new TrapezoidProfile.Constraints(6, 8));
        rollPID.setIZone(sharedIZone);
        rollPID.enableContinuousInput(0, 2 * Math.PI);
        rollPID.setGoal(0);

        differentialArmLogAutoLogged = new DifferentialArmLogAutoLogged();
    }

    private SparkBaseConfig configureMotor() {
        SparkBaseConfig config1 = new SparkMaxConfig().smartCurrentLimit(30);
        config1.encoder.positionConversionFactor(Math.PI / 4);
        return config1;
    }

    @Override
    public void setPitchTarget(DifferentialArmPitch pitchTarget) {
        this.pitchTarget = pitchTarget.rotation;
    }

    @Override
    public void setPitchTarget(double pitchTarget) {
        this.pitchTarget = MathUtils.clamp(MIN_PITCH, MAX_PITCH, pitchTarget);
    }

    @Override
    public void setRollTarget(double rollTarget) {
        this.rollTarget = MathUtils.clamp(MIN_ROLL, MAX_ROLL, rollTarget);
    }

    @Override
    public void runPitch(double sign) {
        setPitchTarget(pitchTarget + PITCH_SPEED * sign);
    }

    @Override
    public void runRoll(double sign) {
        setRollTarget(rollTarget + ROLL_SPEED * sign);
    }

    @Override
    public double getPitch() {
        return (encoder1.getPosition() + encoder2.getPosition()) / 2;
    }

    @Override
    public double getRoll() {
        return ((encoder1.getPosition() - encoder2.getPosition()) / 2) % (2 * Math.PI);
    }

    Map.Entry<Double, Double> normalizePowers(double power1, double power2) {
        double max = Math.max(power1, power2);
        if (max > MAX_POWER) {
            power2 /= max;
            power1 /= max;
        }
        return Map.entry(power1, power2);
    }

    @Override
    public boolean pitchAtSetpoint() {
        return pitchPID.atSetpoint();
    }

    @Override
    public boolean rollAtSetpoint() {
        return rollPID.atSetpoint();
    }

    private double feedforward(double power, double feedForward) {
        if (Math.abs(power) != 0) {
            power += feedForward * Math.signum(power);
        }

        return power;
    }

    @Override
    public void periodic() {
        double pitchCalc = pitchPID.calculate(getPitch(), pitchTarget);
        double rollCalc = rollPID.calculate(getRoll(), rollTarget);

        double power1 = pitchCalc + rollCalc;
        double power2 = pitchCalc - rollCalc;

        power1 = feedforward(power1, FEED_FORWARD1);
        power2 = feedforward(power2, FEED_FORWARD2);

        Map.Entry<Double, Double> powers = normalizePowers(power1, power2);

        motor1.set(powers.getKey());
        motor2.set(powers.getValue());
    }

    @Override
    public LoggableInputs log() {
        differentialArmLogAutoLogged.pitch = getPitch();
        differentialArmLogAutoLogged.roll = getRoll();

        differentialArmLogAutoLogged.pitchTarget = pitchTarget;
        differentialArmLogAutoLogged.rollTarget = rollTarget;

        return differentialArmLogAutoLogged;
    }
}
