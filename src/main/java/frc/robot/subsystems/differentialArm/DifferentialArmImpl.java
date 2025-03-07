package frc.robot.subsystems.differentialArm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.model.DifferentialArmLogAutoLogged;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.SparkFactory;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class DifferentialArmImpl extends SubsystemBase implements DifferentialArmSubsystem {
    private final double PITCH_SPEED = .05;

    private final double MAX_POWER = 0.5;

    private final double MIN_PITCH = 0;
    private final double MAX_PITCH = 2.7;

    private final double sharedP = 0.6;
    private final double sharedI = 0.4;
    private final double sharedD = 0.0025;
    private final double sharedIZone = 0.1;

    private final DifferentialArmLogAutoLogged differentialArmLogAutoLogged;

    SparkMax motor1;
    SparkMax motor2;

    RelativeEncoder encoder1;

    double pitchTarget;

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
                new ProfiledPIDController(
                        sharedP, sharedI, sharedD, new TrapezoidProfile.Constraints(6, 8));
        pitchPID.setIZone(sharedIZone);
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
        this.pitchTarget = pitchTarget.rotation;
    }

    @Override
    public void setPitchTarget(double pitchTarget) {
        this.pitchTarget = MathUtils.clamp(MIN_PITCH, MAX_PITCH, pitchTarget);
    }

    @Override
    public void runPitch(double sign) {
        setPitchTarget(pitchTarget + PITCH_SPEED * sign);
    }

    @Override
    public double getPitch() {
        return encoder1.getPosition();
    }

    @Override
    public boolean pitchAtSetpoint() {
        return pitchPID.atSetpoint();
    }

    @Override
    public void periodic() {
        double pitchCalc = pitchPID.calculate(getPitch(), pitchTarget);

        motor1.set(pitchCalc);
    }

    @Override
    public LoggableInputs log() {
        differentialArmLogAutoLogged.pitch = getPitch();

        differentialArmLogAutoLogged.pitchTarget = pitchTarget;

        return differentialArmLogAutoLogged;
    }
}
