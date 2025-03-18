package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.model.ElevatorLogAutoLogged;
import frc.robot.utilities.SparkFactory;

public class ElevatorNeo extends SubsystemBase implements ElevatorIO {

    SparkMax motor1;
    SparkMax motor2;

    RelativeEncoder motor1Encoder;

    public ElevatorNeo() {
        motor1 = SparkFactory.createSparkMax(Constants.SparkIDs.elevator1);
        motor2 = SparkFactory.createSparkMax(Constants.SparkIDs.elevator2);

        SparkBaseConfig configgled =
                new SparkMaxConfig()
                        .inverted(true)
                        .apply(
                                new SoftLimitConfig()
                                        .forwardSoftLimit(ElevatorSubsystem.MAX)
                                        .forwardSoftLimitEnabled(true)
                                        .reverseSoftLimit(ElevatorSubsystem.MIN)
                                        .reverseSoftLimitEnabled(true));

        motor1.configure(configgled, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motor2.configure(
                new SparkMaxConfig().follow(motor1, true),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

        motor1Encoder = motor1.getEncoder();
    }

	@Override
	public void set(double speed) {
		motor1.set(speed);
	}

	@Override
	public void updateInputs(ElevatorLogAutoLogged inputs) {
		inputs.position = motor1Encoder.getPosition();
	}
}
