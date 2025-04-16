package frc.robot.model;

import frc.robot.subsystems.light.LEDState;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LightLogEntry implements LoggableInputs, Cloneable {

    public String name;

    @Override
    public void toLog(LogTable table) {
        table.put("out", LEDState.out);

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
        table.put("elevatorSetpointBarge", LEDState.elevatorSetpointBarge);
        table.put("algeeIntaking", LEDState.algeeIntaking);
        table.put("name", name);
    }

    @Override
    public void fromLog(LogTable table) {

        LEDState.out = table.get("out", false);

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
        LEDState.elevatorSetpointBarge = table.get("elevatorSetpointBarge", false);
        LEDState.algeeIntaking = table.get("algeeIntaking", false);

        name = table.get("name", name);
    }

    public LightLogEntry clone() {
        LightLogEntry clone = new LightLogEntry();
        clone.name = name;
        return clone;
    }
}
