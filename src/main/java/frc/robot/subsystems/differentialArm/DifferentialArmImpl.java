package frc.robot.subsystems.differentialArm;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.model.DifferentialArmLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.utilities.SparkFactory;
import java.util.Map;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class DifferentialArmImpl extends SubsystemBase
    implements DifferentialArmSubsystem, LoggedSubsystem {
  private final double PITCH_SPEED = 1;
  private final double ROLL_SPEED = 1;

  private final double MAX_POWER = 1;

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

    pitchPID = new PIDController(0.01, 0, 0);
    pitchPID.setSetpoint(0);
    rollPID = new PIDController(0.01, 0, 0);
    rollPID.setSetpoint(0);

    differentialArmLogAutoLogged = new DifferentialArmLogAutoLogged();
  }

  @Override
  public void setPitchTarget(DifferentialArmPitch pitchTarget) {
    this.pitchTarget = pitchTarget.rotation;
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
  public void runPitch(double sign) {
    pitchTarget += PITCH_SPEED * sign;
  }

  @Override
  public void runRoll(double sign) {
    rollTarget += ROLL_SPEED * sign;
  }

  @Override
  public double getPitch() {
    return (encoder1.getPosition() + encoder2.getPosition()) / 2;
  }

  @Override
  public double getRoll() {
    return (encoder1.getPosition() - encoder2.getPosition()) / 2;
  }

  private Map.Entry<Double, Double> normalizePowers(double power1, double power2) {
    if (power1 > MAX_POWER) {
      power2 /= power1;
      power1 /= power1;
    } 

    if (power2 > MAX_POWER) {
      power1 /= power2;
      power2 /= power2;
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

  @Override
  public void periodic() {
    double pitchCalc = pitchPID.calculate(getPitch(), pitchTarget);
    double rollCalc = rollPID.calculate(getRoll(), rollTarget);

    double power1 = pitchCalc + rollCalc;
    double power2 = pitchCalc - rollCalc;

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
