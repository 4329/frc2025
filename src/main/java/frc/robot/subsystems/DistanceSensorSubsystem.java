package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;

public class DistanceSensorSubsystem extends SubsystemBase implements LoggedSubsystem {
	private static final double CORAL_AMOUNT = 500;

	@AutoLog
	public static class DistanceSensorLog {
		double distance;
	}

	ColorSensorV3 distanceSensor = new ColorSensorV3(Port.kOnboard);
	DistanceSensorLogAutoLogged distanceSensorLogAutoLogged = new DistanceSensorLogAutoLogged();


	public boolean getCoraled() {
		return distanceSensor.getProximity() >= CORAL_AMOUNT;
	}

	@Override
	public LoggableInputs log() {
		distanceSensorLogAutoLogged.distance = distanceSensor.getProximity();
		return distanceSensorLogAutoLogged;
	}
}
