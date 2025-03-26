package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.model.AlgeeWheelLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.subsystems.light.LEDState;
import frc.robot.utilities.SparkFactory;
import java.util.Map;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class AlgeeWheelSubsystem extends SubsystemBase implements LoggedSubsystem {
    SparkMax motor1;

    AlgeeWheelLogAutoLogged algeeWheelLogAutoLogged = new AlgeeWheelLogAutoLogged();

    GenericEntry running =
            Shuffleboard.getTab("RobotData")
                    .add("AlgeeWheelRunning", false)
                    .withPosition(4, 2)
                    .withSize(3, 2)
                    .withProperties(Map.of("Color when true", "#00FF00", "Color when false", "#000000"))
                    .getEntry();

    public AlgeeWheelSubsystem() {
        motor1 = SparkFactory.createSparkMax(Constants.SparkIDs.algeeWheel);

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

    @Override
    public void periodic() {
        running.setBoolean(motor1.get() != 0);
        LEDState.algeeWheelRunning = motor1.get() != 0;
    }

    @Override
    public LoggableInputs log() {
        algeeWheelLogAutoLogged.control = motor1.get();

        return algeeWheelLogAutoLogged;
    }
}
