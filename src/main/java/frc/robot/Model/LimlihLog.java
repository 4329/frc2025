package frc.robot.Model;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LimlihLog implements LoggableInputs, Cloneable {

  public class Fiducial {
    public boolean tV;
    public double tX;
    public double tY;
    public Pose3d relativePose;

    public Fiducial(boolean tV, double tX, double tY, Pose3d relativePose) {
      this.tV = tV;
      this.tX = tX;
      this.tY = tY;
      this.relativePose = relativePose;
    }
  }

  public Fiducial[] tags;
  public boolean limlihConnected;
  private final int numTags = 16;

  public LimlihLog() {
    tags = new Fiducial[numTags];
    for (int i = 0; i < tags.length; i++) {
      tags[i] = new Fiducial(false, 0, 0, new Pose3d());
    }
  }

  @Override
  public void toLog(LogTable table) {
    table.put("Limlihconnected", limlihConnected);
    for (int i = 0; i < numTags; i++) {
      LogTable sub = table.getSubtable("" + i);
      sub.put("tV", tags[i].tV);
      sub.put("tX", tags[i].tX);
      sub.put("tY", tags[i].tY);
      Logger.recordOutput("adsf", new Pose2d());
      sub.put("relativePose", tags[i].relativePose);
    }
  }

  @Override
  public void fromLog(LogTable table) {
    limlihConnected = table.get("Limlihconnected", limlihConnected);
    for (int i = 0; i < numTags; i++) {
      LogTable sub = table.getSubtable("" + i);
      tags[i] =
          new Fiducial(
              sub.get("tV").getBoolean(),
              sub.get("tX").getDouble(),
              sub.get("tY").getDouble(),
              sub.get("relativePose", new Pose3d()));
    }
  }

  public LimlihLog clone() {
    LimlihLog copy = new LimlihLog();
    copy.tags = tags.clone();
    copy.limlihConnected = this.limlihConnected;
    return copy;
  }
}
