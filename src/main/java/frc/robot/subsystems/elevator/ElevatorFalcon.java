package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.model.ElevatorLogAutoLogged;

public class ElevatorFalcon extends SubsystemBase implements ElevatorIO {

	TalonFX motor1;
	TalonFX motor2;

	public ElevatorFalcon() {
		motor1 = new TalonFX(Constants.SparkIDs.elevator1);
		motor2 = new TalonFX(Constants.SparkIDs.elevator2);

		motor1.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
		motor2.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
					.withForwardSoftLimitThreshold(ElevatorSubsystem.MAX)
					.withForwardSoftLimitEnable(true)
					.withReverseSoftLimitThreshold(ElevatorSubsystem.MIN)
					.withReverseSoftLimitEnable(true));
		motor2.setControl(new Follower(Constants.SparkIDs.elevator1, true));
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
