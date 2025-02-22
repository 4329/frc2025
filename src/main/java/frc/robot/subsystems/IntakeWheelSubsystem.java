package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SparkFactory;

public class IntakeWheelSubsystem extends SubsystemBase {
    private SparkMax spark;

    public IntakeWheelSubsystem() {
        spark = SparkFactory.createSparkMax(18);
    }

    public void run(double speed) {
        spark.set(speed);
    }

    public void stop() {
        spark.set(0);
    }
}
