package frc.robot.utilities.loggedComands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class LoggedCommandComposer extends Command {
  public LoggedWrapperCommand withNameLog(String name) {
    LoggedWrapperCommand wrapper = new LoggedWrapperCommand(this);
    wrapper.setName(name);
    return wrapper;
  }

  public LoggedParallelRaceGroup raceWithLog(Command... parallel) {
    LoggedParallelRaceGroup group = new LoggedParallelRaceGroup(this.getName(), this);
    group.addCommands(parallel);
    return group;
  }

  public LoggedParallelRaceGroup untilLog(BooleanSupplier condition) {
    return raceWithLog(new WaitUntilCommand(condition));
  }

  public LoggedRepeatCommand repeatedlyLog() {
    return new LoggedRepeatCommand(this);
  }
}
