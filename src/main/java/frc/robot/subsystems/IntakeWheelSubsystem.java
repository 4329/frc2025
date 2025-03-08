package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.SparkFactory;

public class IntakeWheelSubsystem extends SubsystemBase {
    private SparkMax spark;

    public IntakeWheelSubsystem() {
        spark = SparkFactory.createSparkMax(Constants.SparkIDs.intakeWheel);
    }

    public void run(double speed) {
        spark.set(speed);
    }

    public void stop() {
        spark.set(0);
    }
}
