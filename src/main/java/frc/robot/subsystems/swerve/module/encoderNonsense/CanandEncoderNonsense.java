package frc.robot.subsystems.swerve.module.encoderNonsense;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig;

public class CanandEncoderNonsense implements EncoderNonsense {

  private final SparkAbsoluteEncoder m_turningEncoder;

  public CanandEncoderNonsense(SparkBaseConfig conf, SparkAbsoluteEncoder turningEncoder) {
    conf.encoder.positionConversionFactor(2 * Math.PI);
    m_turningEncoder = turningEncoder;
  }

  @Override
  public double get() {
    return -m_turningEncoder.getPosition();
  }
}
