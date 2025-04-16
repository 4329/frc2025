package frc.robot.subsystems.light.ledAnimations;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.light.LightSubsystem;

public class RisingPattern implements LEDPattern {

	@Override
	public void applyTo(LEDReader reader, LEDWriter writer) {
        for (int i = 0; i < LightSubsystem.SIDE_LENGTH; i++) {
            double add = Timer.getFPGATimestamp() * 128;
            int hue = (int)(i * 2 + add);
            writer.setHSV(i, hue, 255, 255);
            writer.setHSV(reader.getLength() - 1 - i, hue, 255, 255);
        }

        for (int i = LightSubsystem.SIDE_LENGTH; i < reader.getLength() - 1 - LightSubsystem.SIDE_LENGTH; i++) {
            if ((int)(Timer.getFPGATimestamp() * 2) % 2 == 0) writer.setRGB(i, 255, 255, 255);
            else writer.setRGB(i, 0, 0, 0);
        }
	}
}
