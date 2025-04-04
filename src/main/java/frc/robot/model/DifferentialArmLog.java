package frc.robot.model;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class DifferentialArmLog {
    public double pitch;
    public double pitchTarget;
    public boolean atSetpoint;
    public double pidCalc;
    public double ffCalc;
}
