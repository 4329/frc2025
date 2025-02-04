package frc.robot.subsystems.light;

import org.littletonrobotics.junction.Logger;

public class LEDState {
  public static boolean on;

  public static int reefButton;
  public static int reefLevel;

  public static boolean centerRunning;

  public static boolean algeeWheelRunning;
  public static boolean algeeWheelHolding;

  public static boolean coralClawHolding;

  public static boolean climbing;

  public static boolean targetVisible;

  public static void log() {
    Logger.recordOutput("LEDState/on", on);

    Logger.recordOutput("LEDState/reefButton", reefButton);
    Logger.recordOutput("LEDState/reefLevel", reefLevel);

    Logger.recordOutput("LEDState/centerRunning", centerRunning);

    Logger.recordOutput("LEDState/algeeWheelRunning", algeeWheelRunning);
    Logger.recordOutput("LEDState/algeeWheelHolding", algeeWheelHolding);

    Logger.recordOutput("LEDState/coralClawHolding", coralClawHolding);

    Logger.recordOutput("LEDState/climbing", climbing);

    Logger.recordOutput("LEDState/targetVisible", targetVisible);
  }
}
