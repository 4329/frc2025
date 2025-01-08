package frc.robot.utilities;

public enum ShotRpms {
  PASS(2750),
  REV(3300);

  private int value;

  private ShotRpms(int adsf) {

    this.value = adsf;
  }

  public int getValue() {

    return value;
  }
}
