package frc.robot.subsystems.light.ledAnimations;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.Timer;

public class SinPattern implements LEDPattern {

  double period = 4.0;
  double amplitude = 150;
  double baseLine = 0;
  double waveSpeed = 20;
  double pulseSpeed = 3;
  double scaleFactor = 1.0;

  @Override
  public void applyTo(LEDReader reader, LEDWriter writer) {
    double offset = Timer.getFPGATimestamp() * waveSpeed / reader.getLength();

    for (int i = 0; i < reader.getLength(); i++) {
      double initialI = ((double) i) / (double) reader.getLength();
      double sinish = Math.sin((initialI + offset) * (2 * Math.PI) * period);
      double saturation = (sinish + 1.0) * amplitude / 2.0;

      double pulseI =
          (Math.sin(Timer.getFPGATimestamp() * pulseSpeed) + 1.0) * ((255.0 - amplitude) / 2.0);

      writer.setHSV(i, 7, 255, (int) Math.round((saturation + pulseI) * scaleFactor));
    }
  }
}
