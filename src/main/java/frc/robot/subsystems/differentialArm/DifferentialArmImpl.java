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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.model.DifferentialArmLogAutoLogged;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.SparkFactory;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import frc.robot.utilities.shufflebored.*;

public class DifferentialArmImpl extends SubsystemBase implements DifferentialArmSubsystem {
    private final double PITCH_SPEED = .05;

    private final double MIN_PITCH = 0;
    private final double MAX_PITCH = 2.95;

    private final DifferentialArmLogAutoLogged differentialArmLogAutoLogged;

    SparkMax motor1;
    SparkMax motor2;

    RelativeEncoder encoder1;

    ProfiledPIDController pitchPID;

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
                new ShuffledTrapezoidController(0.2, 1, 0.0025, new TrapezoidProfile.Constraints(6, 8));
        pitchPID.setIZone(0.3);
		pitchPID.setTolerance(0.1);
        pitchPID.setGoal(0);

        Shuffleboard.getTab("Asdf").add("diff", pitchPID);

        differentialArmLogAutoLogged = new DifferentialArmLogAutoLogged();
    }

    private SparkBaseConfig configureMotor() {
        SparkBaseConfig config1 = new SparkMaxConfig().smartCurrentLimit(30);
        config1.encoder.positionConversionFactor(Math.PI / 4);
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
    public boolean pitchAtSetpoint() {
        return pitchPID.atGoal();
    }

    @Override
    public void periodic() {
        motor1.set(pitchPID.calculate(getPitch()));
    }

	@Override
	public void voltageDrive(Voltage voltage) {
		motor1.setVoltage(voltage);
	}

	@Override
	public void logMotors(SysIdRoutineLog log) {
	}

    @Override
    public LoggableInputs log() {
        differentialArmLogAutoLogged.pitch = getPitch();
        differentialArmLogAutoLogged.pitchTarget = pitchPID.getGoal().position;
		differentialArmLogAutoLogged.atSetpoint = pitchAtSetpoint();

        return differentialArmLogAutoLogged;
    }
}
