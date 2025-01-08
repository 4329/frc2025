package frc.robot.utilities;

public enum ClimberSetpoints {
  ZERO(0),
  CLIMBUP(5),
  // LEFTUP(110),
  // LEFTCLIMB(25.43f),
  CLIMBED(5f),
  UPMAX(200f),
  LEFTMAX(205f),
  // RIGHTUP(110f),
  // RIGHTCLIMBED(28.09f),
  RIGHTMAX(227.25f);

  private float value;

  private ClimberSetpoints(float sam) {

    this.value = sam;
  }

  public float getValue() {

    return value;
  }
}
