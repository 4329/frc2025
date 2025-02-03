package frc.robot.subsystems.light.ledAnimations;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class BeamsPattern implements LEDPattern {
    double beamSpeed = 20f;
    double beamDst = 4;
    double numBeams = 8;

    @Override
    public void applyTo(LEDReader reader, LEDWriter writer) {
            double offset = Timer.getFPGATimestamp() * beamSpeed;
            int cutOffset = (int)Math.round(offset);
          
            for (int i = 0; i < reader.getLength(); i++) {
              int dst = cutOffset - i;
              if (Math.abs(dst) % numBeams <= beamDst) {
                writer.setLED(i, Color.kMagenta);
              } else {
                writer.setLED(i, Color.kBlack);
              }
            }
    }
    
}
