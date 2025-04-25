package frc.robot.model;

import frc.robot.subsystems.light.LEDState;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LightLogEntry implements LoggableInputs, Cloneable {

    public String name;

    @Override
    public void toLog(LogTable table) {
        table.put("out", LEDState.out);
        table.put("teleoped", LEDState.teleoped);

        table.put("byHpStation", LEDState.byHpStation);
        table.put("byReef", LEDState.byReef);
        table.put("byPorcessor", LEDState.byPorcessor);
        table.put("byBarge", LEDState.byBarge);

        table.put("algeeWheelRunning", LEDState.algeeWheelRunning);
        table.put("elevatorAtSetpoint", LEDState.elevatorAtSetpoint);
        table.put("centered", LEDState.centered);
        table.put("scoreCoral", LEDState.scoreCoral);

        table.put("scoreWithArm", LEDState.scoreWithArm);
        table.put("porcessor", LEDState.porcessor);
        table.put("elevatorSetpoint", LEDState.elevatorSetpoint);
        table.put("algeeIntaking", LEDState.algeeIntaking);
        table.put("name", name);
    }

    @Override
    public void fromLog(LogTable table) {

        LEDState.out = table.get("out", false);
        LEDState.teleoped = table.get("teleoped", false);

        LEDState.byHpStation = table.get("byHpStation", false);
        LEDState.byReef = table.get("byReef", false);
        LEDState.byPorcessor = table.get("byPorcessor", false);
        LEDState.byBarge = table.get("byBarge", false);

        LEDState.algeeWheelRunning = table.get("algeeWheelRunning", false);
        LEDState.elevatorAtSetpoint = table.get("elevatorAtSetpoint", false);
        LEDState.centered = table.get("centered", false);
        LEDState.scoreCoral = table.get("scoreCoral", false);

        LEDState.scoreWithArm = table.get("scoreWithArm", false);
        LEDState.porcessor = table.get("porcessor", false);
        LEDState.elevatorSetpoint = table.get("elevatorSetpoint", 0);
        LEDState.algeeIntaking = table.get("algeeIntaking", false);

        name = table.get("name", name);
    }

    public LightLogEntry clone() {
        LightLogEntry clone = new LightLogEntry();
        clone.name = name;
        return clone;
    }
}
