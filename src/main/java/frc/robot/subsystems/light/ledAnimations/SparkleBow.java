package frc.robot.subsystems.light.ledAnimations;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class SparkleBow implements LEDPattern {

    final int NUM_FLASHIES = 1;
    private int speed;

    public SparkleBow(int speed) {
        this.speed = speed;
    }

    @Override
    public void applyTo(LEDReader reader, LEDWriter writer) {
        int time = (int)(Timer.getFPGATimestamp() * speed);
        for (int i = 0; i < reader.getLength(); i++) {
            writer.setHSV(i, i  * 4 + time, 255, 255);
        }
        for (int i = 0; i < NUM_FLASHIES; i++) {
            int index = (int)(Math.random() * reader.getLength());
            writer.setLED(index, Color.lerpRGB(reader.getLED(index), Color.kWhite, 0.9));
        }
    }
    
}
