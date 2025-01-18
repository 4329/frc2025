package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;

public class UnInstantCommand extends Command {

  Runnable toRun;

  public UnInstantCommand(Runnable toRun) {
    this.toRun = toRun;
  }

  @Override
  public void execute() {
    toRun.run();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
