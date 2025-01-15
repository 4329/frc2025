package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LoggingSubsystem extends SubsystemBase {

  private LoggedSubsystem[] subsystems;
  private boolean isEven;
  private int timer;

  public LoggingSubsystem(LoggedSubsystem... subsystems) {
    this.subsystems = subsystems;
  }

  @Override
  public void periodic() {
    timer++;

    if (timer % 3 == 0) {
      for (int i = isEven ? 0 : 1; i < subsystems.length; i += 2) {
        String name = subsystems[i].getClass().getSimpleName();
        Logger.processInputs(name, subsystems[i].log());
      }
      isEven = !isEven;
    }
  }

  public static interface LoggedSubsystem {

    public LoggableInputs log();
  }
}
