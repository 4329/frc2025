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
  SparkMax motor2;

  public AlgeeWheelSubsystem() {
    motor1 = SparkFactory.createSparkMax(13);
    motor2 = SparkFactory.createSparkMax(14);

    motor2.configure(
        new SparkMaxConfig().follow(motor1),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void run(double speed) {
    motor1.set(speed);
  }

  @Override
  public void periodic() {
    LEDState.algeeWheelRunning = Math.abs(motor1.get()) != 0;
  }
}
