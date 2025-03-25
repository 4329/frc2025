package frc.robot.model;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LilihLog implements LoggableInputs, Cloneable {

    public class Fiducial {
        public boolean tV;
        public double tX;
        public double tY;
		public double tA;

        public Fiducial(boolean tV, double tX, double tY, double tA) {
            this.tV = tV;
            this.tX = tX;
            this.tY = tY;
			this.tA = tA;
        }
    }

    public Fiducial[] tags;
    public boolean limlihConnected;
    public static final int NUM_TAGS = 22;

    public LilihLog() {
        tags = new Fiducial[NUM_TAGS];
        for (int i = 0; i < tags.length; i++) {
            tags[i] = new Fiducial(false, 0, 0, 0);
        }
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Limlihconnected", limlihConnected);
        for (int i = 0; i < NUM_TAGS; i++) {
            LogTable sub = table.getSubtable("" + (i + 1));
            sub.put("tV", tags[i].tV);
            sub.put("tX", tags[i].tX);
            sub.put("tY", tags[i].tY);
            sub.put("tA", tags[i].tA);
        }
    }

    @Override
    public void fromLog(LogTable table) {
        limlihConnected = table.get("Limlihconnected", limlihConnected);
        for (int i = 0; i < NUM_TAGS; i++) {
            LogTable sub = table.getSubtable("" + (i + 1));
            tags[i] =
                    new Fiducial(
                            sub.get("tV").getBoolean(),
                            sub.get("tX").getDouble(),
                            sub.get("tY").getDouble(),
                            sub.get("tA").getDouble());
        }
    }

    public LilihLog clone() {
        LilihLog copy = new LilihLog();
        copy.tags = tags.clone();
        copy.limlihConnected = this.limlihConnected;
        return copy;
    }
}
