package frc.robot.Model;

import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class CommandLogEntry implements LoggableInputs, Cloneable {

    public Map<String, String> commands;

    public CommandLogEntry() {
        commands = new HashMap<>();
    }

    @Override
    public void toLog(LogTable table) {
        commands.forEach(
                (String name, String message) -> {
                    table.put(name, message);
                });
    }

    @Override
    public void fromLog(LogTable table) {
        commands.forEach(
                (String name, String message) -> {
                    commands.replace(name, table.get(name, message));
                });
    }

    public CommandLogEntry clone() {
        CommandLogEntry copy = new CommandLogEntry();
        copy.commands = new HashMap<>(commands);
        return copy;
    }

    public void set(String name, String message) {
        if (!commands.containsKey(name)) commands.put(name, message);
        else commands.replace(name, message);
    }
}
