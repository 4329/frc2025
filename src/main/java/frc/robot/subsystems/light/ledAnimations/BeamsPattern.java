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
    Color color1;
    Color color2;

    public BeamsPattern(Color color1, Color color2) {
        this.color1 = color1;
        this.color2 = color2;
    }

    @Override
    public void applyTo(LEDReader reader, LEDWriter writer) {
        double offset = Timer.getFPGATimestamp() * beamSpeed;
        int cutOffset = (int) Math.round(offset);

        for (int i = 0; i < reader.getLength(); i++) {
            int dst = cutOffset - i;

            if (Math.abs(dst) % numBeams <= beamDst) writer.setLED(i, color1);
            else writer.setLED(i, color2);
        }
    }
}
