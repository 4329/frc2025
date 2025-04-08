package frc.robot.subsystems.light.ledAnimations;

import java.util.TimeZone;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.Timer;

public class PolicePattern implements LEDPattern {
    int policeDst = 4;
    int numPolice = 4;
    int count = 0;

    @Override
    public void applyTo(LEDReader reader, LEDWriter writer) {

        int paneLength = policeDst + numPolice;
        int numPanes = reader.getLength()/paneLength + 1;

        for (int i = 0; i < reader.getLength(); i++) {
            writer.setRGB(i, 255, 255, 255);
        }

        int time = count++;
        System.out.println(time);
        for (int i = 0;  i < numPanes; i++) {
            int add = i * paneLength + time;
            for (int j = 0; j + add < reader.getLength() && j < numPolice; j++) {
                writer.setRGB(j + add, 0, 0, 0);
            }
        }
    }
}
