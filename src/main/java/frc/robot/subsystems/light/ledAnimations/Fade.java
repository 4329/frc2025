package frc.robot.subsystems.light.ledAnimations;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.Timer;

public class Fade implements LEDPattern {

    double time = 0.2;
    Timer timer = new Timer();

    public Fade(double time) {
        this.time = time;
    }

    @Override
    public void applyTo(LEDReader reader, LEDWriter writer) {
        for (int i = 0; i < reader.getLength(); i++) {
        }
    }
    
}
