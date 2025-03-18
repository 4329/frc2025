package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.model.ElevatorLogAutoLogged;

public class ElevatorFalcon extends SubsystemBase implements ElevatorIO {

	TalonFX motor1;
	TalonFX motor2;

	public ElevatorFalcon() {
		motor1 = new TalonFX(Constants.SparkIDs.elevator1);
		motor2 = new TalonFX(Constants.SparkIDs.elevator2);

		motor2.getConfigurator().apply(new TalonFXConfiguration());
	}

	@Override
	public void set(double speed) {
		motor1.set(speed);
	}

	@Override
	public void updateInputs(ElevatorLogAutoLogged inputs) {
		inputs.position = motor1.getPosition().getValue().magnitude();
	}
}
