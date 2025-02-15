package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.light.LEDState;
import frc.robot.utilities.SparkFactory;

public class AlgeeWheelSubsystem extends SubsystemBase {
    SparkMax motor1;

    private double min = -10;
    private double max = 10;

    public AlgeeWheelSubsystem() {
        motor1 = SparkFactory.createSparkMax(90);

        motor1.configure(
                new SparkMaxConfig()
                        .apply(
                                new SoftLimitConfig()
                                        .forwardSoftLimit(10)
                                        .forwardSoftLimitEnabled(true)
                                        .reverseSoftLimit(-10)
                                        .reverseSoftLimitEnabled(true)),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void run(double speed) {
        motor1.set(speed);
    }

    public void stop() {
        motor1.set(0);
    }

    @Override
    public void periodic() {
        LEDState.algeeWheelRunning = motor1.get() != 0;
    }
}
