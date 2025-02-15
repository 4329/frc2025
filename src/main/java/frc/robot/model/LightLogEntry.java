package frc.robot.model;

import frc.robot.subsystems.light.LEDState;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LightLogEntry implements LoggableInputs, Cloneable {

    public String name;

    @Override
    public void toLog(LogTable table) {
        table.put("State/on", LEDState.on);

        table.put("State/reefButton", LEDState.reefButton);
        table.put("State/reefLevel", LEDState.reefLevel);

        table.put("State/centerRunning", LEDState.centerRunning);

        table.put("State/algeeWheelRunning", LEDState.algeeWheelRunning);
        table.put("State/algeeWheelHolding", LEDState.algeeWheelHolding);

        table.put("State/coralClawHolding", LEDState.coralClawHolding);

        table.put("State/climbing", LEDState.climbing);

        table.put("State/targetVisible", LEDState.targetVisible);

        table.put("name", name);
    }

    @Override
    public void fromLog(LogTable table) {
        LEDState.on = table.get("State/on", false);

        LEDState.reefButton = table.get("State/reefButton", 0);
        LEDState.reefLevel = table.get("State/reefLevel", 0);

        LEDState.centerRunning = table.get("State/centerRunning", false);

        LEDState.algeeWheelRunning = table.get("State/algeeWheelRunning", false);
        LEDState.algeeWheelHolding = table.get("State/algeeWheelHolding", false);

        LEDState.coralClawHolding = table.get("State/coralClawHolding", false);

        LEDState.climbing = table.get("State/climbing", false);

        LEDState.targetVisible = table.get("State/targetVisible", false);

        name = table.get("name", name);
    }

    public LightLogEntry clone() {
        LightLogEntry clone = new LightLogEntry();
        clone.name = name;
        return clone;
    }
}
