package frc.robot.subsystems.differentialArm;

import java.util.Map;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.model.DifferentialArmLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.utilities.SparkFactory;

public class DifferentialArmImpl extends SubsystemBase implements DifferentialArmSubsystem, LoggedSubsystem {
    private final double MAX_SPEED = 1;
    private final DifferentialArmLogAutoLogged differentialArmLogAutoLogged;

    SparkMax motor1;
    SparkMax motor2;

    RelativeEncoder encoder1;
    RelativeEncoder encoder2;

    double pitchTarget;
    double rollTarget;

    PIDController pitchPID;
    PIDController rollPID;

    public DifferentialArmImpl() {
        motor1 = SparkFactory.createSparkMax(9);
        motor2 = SparkFactory.createSparkMax(10);

        encoder1 = motor1.getEncoder();
        encoder2 = motor2.getEncoder();

        pitchPID = new PIDController(0.20, 0, 0);
        pitchPID.setSetpoint(0);
        rollPID = new PIDController(0.01, 0, 0);
        rollPID.setSetpoint(0);

        differentialArmLogAutoLogged = new DifferentialArmLogAutoLogged();
    }

    @Override
    public void setPitchTarget(double pitchTarget) {
        this.pitchTarget = pitchTarget;
    }

    @Override
    public void setRollTarget(double rollTarget) {
        this.rollTarget = rollTarget;
    }

    @Override
    public double getPitch() {
        return (encoder1.getPosition() + encoder2.getPosition()) / 2;
    }

    @Override
    public double getRoll() {
        return encoder1.getPosition() - encoder2.getPosition();
    }

    private Map.Entry<Double, Double> normalizeSpeeds(double speed1, double speed2) {
        if (speed1 > MAX_SPEED) {
            speed1 /= speed1;
            speed2 /= speed1;
        } else if (speed2 > MAX_SPEED) {
            speed1 /= speed2;
            speed2 /= speed2;
        }

        return Map.entry(speed1, speed2);
    }

    @Override
    public boolean pitchAtSetpoint() {
        return pitchPID.atSetpoint();
    }

    @Override
    public boolean rollAtSetpoint() {
        return rollPID.atSetpoint();
    }

    @Override
    public void periodic() {
        double pitchCalc = pitchPID.calculate(getPitch(), pitchTarget);
        double rollCalc = rollPID.calculate(getRoll(), rollTarget);

        double speed1 = pitchCalc + rollCalc;
        double speed2 = pitchCalc - rollCalc;

        Map.Entry<Double, Double> speeds = normalizeSpeeds(speed1, speed2);

        motor1.set(speeds.getKey());
        motor2.set(speeds.getValue());
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