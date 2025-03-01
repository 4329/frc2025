package frc.robot.model;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.loggedComands.LoggedSequentialCommandGroup;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class CommandLogEntry implements LoggableInputs, Cloneable {

    public Map<Command, String> commands;

    public CommandLogEntry() {
        commands = new HashMap<>();
    }

    @Override
    public void toLog(LogTable table) {
        commands.forEach(
                (Command command, String message) -> {
                    if (command instanceof LoggableInputs) {
                        ((LoggableInputs) command).toLog(table.getSubtable(command.getName()));
                    }
                    table.put(command.getName(), message);
                });
    }

    @Override
    public void fromLog(LogTable table) {
        commands.forEach(
                (Command command, String message) -> {
                    if (command instanceof LoggedSequentialCommandGroup) {
                        ((LoggedSequentialCommandGroup) command).fromLog(table.getSubtable(command.getName()));
                    }
                    commands.put(command, table.get(command.getName(), message));
                });
    }

    public CommandLogEntry clone() {
        CommandLogEntry copy = new CommandLogEntry();
        copy.commands = new HashMap<>(commands);
        return copy;
    }

    public void put(Command name, String message) {
        commands.put(name, message);
    }
}
