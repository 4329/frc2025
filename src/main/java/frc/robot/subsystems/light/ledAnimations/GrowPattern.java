package frc.robot.subsystems.light.ledAnimations;

import java.io.Serial;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.Timer;

public class GrowPattern implements LEDPattern {
    double growHeight = 0.5;
    double growFloor = 0.2;
    double growSpeed = 2;

    @Override
    public void applyTo(LEDReader reader, LEDWriter writer) {
        double mult = Math.sin(Timer.getFPGATimestamp() * growSpeed);
        double scaledMult = ((mult + 1) / 2.0) * growHeight + growFloor;

        for (int i = 0; i < reader.getLength(); i++) {
            double dst = (reader.getLength() / 2.0f) - i;
            double scaledDst = 1 - (Math.abs(dst) / (reader.getLength() / 2.0));
            scaledDst = (scaledDst - scaledMult) / (1 - scaledMult);
            scaledDst = scaledDst < 0 ? 0 : scaledDst;
            scaledDst = Math.pow(scaledDst, 1.0f);

            writer.setHSV(i, 90, 255, (int)Math.round(scaledDst * 255.0));
        }
    }

    
}
