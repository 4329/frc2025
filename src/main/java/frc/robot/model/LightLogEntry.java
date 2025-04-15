package frc.robot.model;

import frc.robot.subsystems.light.LEDState;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LightLogEntry implements LoggableInputs, Cloneable {

    public String name;

    @Override
    public void toLog(LogTable table) {
        table.put("State/centerRunning", LEDState.centerRunning);
        table.put("State/targetVisible", LEDState.targetVisible);

        table.put("name", name);
    }

    @Override
    public void fromLog(LogTable table) {
        LEDState.centerRunning = table.get("State/centerRunning", false);
        LEDState.targetVisible = table.get("State/targetVisible", false);

        name = table.get("name", name);
    }

    public LightLogEntry clone() {
        LightLogEntry clone = new LightLogEntry();
        clone.name = name;
        return clone;
    }
}
