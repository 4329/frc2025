package frc.robot.subsystems.swerve.module.encoderNonsense;

import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class ThriftyEncoder implements EncoderNonsense {

  private final AnalogPotentiometer m_turningEncoder;

  public ThriftyEncoder(int turningEncoderChannel) {
    m_turningEncoder = new AnalogPotentiometer(turningEncoderChannel, 2 * Math.PI);
  }

  @Override
  public double get() {
    return m_turningEncoder.get();
  }
}
