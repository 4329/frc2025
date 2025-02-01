package frc.robot.subsystems.light;

public class LEDState {
  public static boolean on;

  public static int reefButton;
  public static int reefLevel;

  public static boolean centerRunning;

  public static boolean agleeWheelRunning;
  public static boolean algeeWheelHolding;

  public static boolean coralClawHolding;

  public static boolean climbing;

  public static boolean targetVisible;

  public static String asString() {
    return String.format("""
        on: %b,
        reefButton: %d,
        reefLevel: %d,
        centerRunning: %b,
        algeeWheelRunning: %b,
        algeeWheelHolding: %b,
        coralClawHolding: %b,
        climbing: %b,
        targetVisible: %b,
        """,
        on,
        reefButton,
        reefLevel,
        centerRunning,
        agleeWheelRunning,
        algeeWheelHolding,
        coralClawHolding,
        climbing,
        targetVisible);
  }
}
