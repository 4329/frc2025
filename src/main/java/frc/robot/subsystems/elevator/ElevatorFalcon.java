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
import frc.robot.utilities.MathUtils;

public class ElevatorFalcon extends SubsystemBase implements ElevatorIO {

    TalonFX motor1;
    TalonFX motor2;

    double offset = 0;

    public ElevatorFalcon() {
        motor1 = new TalonFX(Constants.SparkIDs.elevator1);
        motor2 = new TalonFX(Constants.SparkIDs.elevator2);

        motor1
                .getConfigurator()
                .apply(
                        new TalonFXConfiguration()
                                .withMotorOutput(
                                        new MotorOutputConfigs()
                                                .withInverted(InvertedValue.Clockwise_Positive)
                                                .withNeutralMode(NeutralModeValue.Brake)));

        motor2
                .getConfigurator()
                .apply(
                        new TalonFXConfiguration()
                                .withSoftwareLimitSwitch(
                                        new SoftwareLimitSwitchConfigs()
                                                .withForwardSoftLimitThreshold(ElevatorSubsystem.MAX)
                                                .withForwardSoftLimitEnable(true)
                                                .withReverseSoftLimitThreshold(ElevatorSubsystem.MIN)
                                                .withReverseSoftLimitEnable(true))
                                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)));

        motor2.setControl(new Follower(Constants.SparkIDs.elevator1, true));

        offset = get();
    }

    private double get() {
        return motor1.getPosition().getValue().magnitude() - offset;
    }

    @Override
    public void set(double speed) {
        motor1.set(MathUtils.clamp(-1, 1, speed));
    }

    @Override
    public void updateInputs(ElevatorLogAutoLogged inputs) {
        inputs.position = get();
    }
}
