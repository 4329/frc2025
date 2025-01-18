package frc.robot.subsystems.swerve.module.encoderNonsense;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig;

public class ReduxEncoder implements EncoderNonsense {

  private final SparkAbsoluteEncoder m_turningEncoder;

  public ReduxEncoder(SparkBaseConfig conf, SparkAbsoluteEncoder turningEncoder) {
    conf.encoder.positionConversionFactor(2 * Math.PI);
    m_turningEncoder = turningEncoder;
  }

  @Override
  public double get() {
    return -m_turningEncoder.getPosition();
  }
}
