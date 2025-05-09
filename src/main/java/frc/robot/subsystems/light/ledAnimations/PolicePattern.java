package frc.robot.subsystems.light.ledAnimations;

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
        int numPanes = reader.getLength() / paneLength + 1;

        for (int i = 0; i < reader.getLength(); i++) {
            writer.setRGB(i, 255, 255, 255);
        }

        double time = Timer.getFPGATimestamp() * 2;
        for (int i = 0; i < numPanes; i++) {
            double index = (i + time) % (numPanes);
            int add = (int) (index * paneLength) - paneLength;

            for (int j = 0; j < numPolice; j++) {
                if (j + add < 0 || j + add >= reader.getLength()) continue;

                if ((i & 1) == 0) {
                    writer.setRGB(j + add, 255, 0, 0);
                } else {
                    writer.setRGB(j + add, 0, 0, 255);
                }
            }
        }
    }
}
