package frc.robot.subsystems.swerve.module.encoderNonsense;

import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class WhateverTheOtherEncoderNonsenseWasCalled implements EncoderNonsense {

  private final AnalogPotentiometer m_turningEncoder;

  public WhateverTheOtherEncoderNonsenseWasCalled(int turningEncoderChannel) {
    m_turningEncoder = new AnalogPotentiometer(turningEncoderChannel, 2 * Math.PI);
  }

  @Override
  public double get() {
    return m_turningEncoder.get();
  }
}
