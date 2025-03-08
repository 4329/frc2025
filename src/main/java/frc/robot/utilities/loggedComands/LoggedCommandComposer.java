package frc.robot.utilities.loggedComands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.BooleanSupplier;

public class LoggedCommandComposer extends Command {
    public LoggedWrapperCommand withNameLog(String name) {
        LoggedWrapperCommand wrapper = new LoggedWrapperCommand(this) {};

        wrapper.setName(name);
        return wrapper;
    }

    public LoggedSequentialCommandGroup andThenLog(Command next) {
        return new LoggedSequentialCommandGroup(this.getName() + "," + next.getName(), this, next);
    }

    public LoggedParallelRaceGroup raceWithLog(String name, Command... parallel) {
        LoggedParallelRaceGroup group = new LoggedParallelRaceGroup(name, this);
        group.addCommands(parallel);
        return group;
    }

    public LoggedParallelRaceGroup untilLog(BooleanSupplier condition) {
        return raceWithLog("Until(" + getName() + ")", new WaitUntilCommand(condition));
    }

    public LoggedRepeatCommand repeatedlyLog() {
        return new LoggedRepeatCommand(this);
    }

    public LoggedWrapperCommand ignoringDisableLog(boolean doesRunWhenDisabled) {
        return new LoggedWrapperCommand(this) {
            @Override
            public boolean runsWhenDisabled() {
                return doesRunWhenDisabled;
            }
        };
    }

    public LoggedWrapperCommand whileLog(BooleanSupplier condition) {
        return new LoggedWrapperCommand(this) {
            @Override
            public boolean isFinished() {
                return !condition.getAsBoolean();
            }
        };
    }
}
