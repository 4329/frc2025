package frc.robot.model;

import java.util.HashMap;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.subsystems.light.LEDState;

public class LightLogEntry implements LoggableInputs, Cloneable {

	public String name;

	@Override
	public void toLog(LogTable table) {
		table.put("on", LEDState.on);

		table.put("reefButton", LEDState.reefButton);
		table.put("reefLevel", LEDState.reefLevel);

		table.put("centerRunning", LEDState.centerRunning);

		table.put("algeeWheelRunning", LEDState.algeeWheelRunning);
		table.put("algeeWheelHolding", LEDState.algeeWheelHolding);

		table.put("coralClawHolding", LEDState.coralClawHolding);

		table.put("climbing", LEDState.climbing);

		table.put("targetVisible", LEDState.targetVisible);

		table.put("name", name);
	}

	@Override
	public void fromLog(LogTable table) {
		LEDState.on = table.get("on", false);

		LEDState.reefButton = table.get("reefButton", 0);
		LEDState.reefLevel = table.get("reefLevel", 0);

		LEDState.centerRunning = table.get("centerRunning", false);

		LEDState.algeeWheelRunning = table.get("algeeWheelRunning", false);
		LEDState.algeeWheelHolding = table.get("algeeWheelHolding", false);

		LEDState.coralClawHolding = table.get("coralClawHolding", false);

		LEDState.climbing = table.get("climbing", false);

		LEDState.targetVisible = table.get("targetVisible", false);

		name = table.get("name", name);
	}

	public LightLogEntry clone() {
		return new LightLogEntry();
	}
}
