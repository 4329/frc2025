package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.light.LEDState;
import frc.robot.utilities.SparkFactory;

public class AlgeeWheelSubsystem extends SubsystemBase {
    SparkMax motor1;

    public AlgeeWheelSubsystem() {
        motor1 = SparkFactory.createSparkMax(10);

        motor1.configure(
                new SparkMaxConfig().inverted(true),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void run(double speed) {
        motor1.set(speed);
    }

    public void stop() {
        motor1.set(0);
    }

    public boolean getAlgeed() {
        return motor1.getOutputCurrent() > 100;
    }

    @Override
    public void periodic() {
        LEDState.algeeWheelRunning = motor1.get() != 0;
    }
}
