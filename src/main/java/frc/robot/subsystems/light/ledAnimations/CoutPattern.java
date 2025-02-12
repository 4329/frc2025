package frc.robot.subsystems.light.ledAnimations;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class CoutPattern implements LEDPattern {
    double coutSpeed = 20;
    double coutDst = 4;
    double numCouts = 8;

    @Override
    public void applyTo(LEDReader reader, LEDWriter writer) {
        double offset = Timer.getFPGATimestamp() * coutSpeed;
        int cutOffset = (int) Math.round(offset);

        for (int i = 0; i < (reader.getLength()) / 2; i++) {
            if (i >= reader.getLength()) continue;
            int dst = cutOffset - (i / 2);
            if (Math.abs(dst) % numCouts <= coutDst) {
                writer.setLED(i, Color.kRed);
                writer.setLED((reader.getLength() - 1) - i, Color.kBlue);
            } else {
                writer.setLED(i, Color.kBlack);
                writer.setLED((reader.getLength() - 1) - i, Color.kBlack);
            }
        }
    }
}
