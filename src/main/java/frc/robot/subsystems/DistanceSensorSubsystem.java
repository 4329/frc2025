package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class DistanceSensorSubsystem extends SubsystemBase {
    @AutoLog
    public static class DistanceSensorLog {
        boolean coraled;
    }

    DistanceSensorLogAutoLogged distanceSensorLogAutoLogged = new DistanceSensorLogAutoLogged();
    DigitalInput pico = new DigitalInput(0);

    public boolean getCoraled() {
        return pico.get();
    }

    @Override
    public void periodic() {
        distanceSensorLogAutoLogged.coraled = pico.get();
        Logger.processInputs("DistanceSensorSubsystem", distanceSensorLogAutoLogged);
    }
}
