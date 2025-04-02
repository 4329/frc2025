package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.PicoColorSensor;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class DistanceSensorSubsystem extends SubsystemBase {
    private static final double CORAL_AMOUNT = 175;

    @AutoLog
    public static class DistanceSensorLog {
        double distance;
    }

    PicoColorSensor distanceSensor = new PicoColorSensor();
    DistanceSensorLogAutoLogged distanceSensorLogAutoLogged = new DistanceSensorLogAutoLogged();

    public boolean getCoraled() {
        distanceSensorLogAutoLogged.distance = distanceSensor.getProximity0();
        Logger.processInputs("DistanceSensorSubsystem", distanceSensorLogAutoLogged);

        return distanceSensorLogAutoLogged.distance >= CORAL_AMOUNT;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("IsSensorConnected", distanceSensor.isSensor0Connected());
        getCoraled();
    }
}
