package frc.robot.utilities;

public enum ElevatorSetpoints {
  ZERO(0),
  INTAKE(10),
  SOURCE(0),
  SUBWOOF(106),
  AMPPOINT(108),
  FULL(109),
  HIGHLIMIT(110);

  private float value;

  private ElevatorSetpoints(float jim) {

    this.value = jim;
  }

  public float getValue() {

    return value;
  }
}
