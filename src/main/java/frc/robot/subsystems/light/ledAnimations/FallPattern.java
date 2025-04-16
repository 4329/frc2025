package frc.robot.subsystems.light.ledAnimations;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.Timer;

public class FallPattern implements LEDPattern {

	@Override
	public void applyTo(LEDReader reader, LEDWriter writer) {
        final int halfLength = reader.getLength() / 2;
        for (int i = 0; i < halfLength; i++) {
            final int index = (int)((i - Timer.getFPGATimestamp() * 5) % (halfLength) + halfLength) % halfLength;
            if (i < 25) {
                writer.setRGB(index, 0, 0, 0);
                writer.setRGB((reader.getLength() - 1) - index, 0, 0, 0);
            } else {
                writer.setRGB(index, 255, 255, 255);
                writer.setRGB((reader.getLength() - 1) - index, 255, 255, 255);
            }
        }
	}
    
}
