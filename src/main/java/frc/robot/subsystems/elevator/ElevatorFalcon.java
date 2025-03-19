package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.model.ElevatorLogAutoLogged;

public class ElevatorFalcon extends SubsystemBase implements ElevatorIO {

    TalonFX motor1;
    TalonFX motor2;

    public ElevatorFalcon() {
        motor1 = new TalonFX(Constants.SparkIDs.elevator1);
        motor2 = new TalonFX(Constants.SparkIDs.elevator2);

        motor1
                .getConfigurator()
                .apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive).withNeutralMode(NeutralModeValue.Brake));

        motor2
                .getConfigurator()
                .apply(new TalonFXConfiguration()
						.withSoftwareLimitSwitch(
							new SoftwareLimitSwitchConfigs()
							.withForwardSoftLimitThreshold(ElevatorSubsystem.MAX)
							.withForwardSoftLimitEnable(true)
							.withReverseSoftLimitThreshold(ElevatorSubsystem.MIN)
							.withReverseSoftLimitEnable(true))
						.withMotorOutput(
							new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)));
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
