package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.SparkFactory;

public class IntakePivotSubsystem extends SubsystemBase {
    private SparkMax spark;

    private PIDController pidController;

    public IntakePivotSubsystem() {
        spark = SparkFactory.createSparkMax(Constants.SparkIDs.intakePivot);

        spark.configure(
                new SparkMaxConfig()
                        .apply(
                                new SoftLimitConfig()
                                        .forwardSoftLimit(10)
                                        .forwardSoftLimitEnabled(true)
                                        .reverseSoftLimit(-10)
                                        .reverseSoftLimitEnabled(true)),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

        pidController = new PIDController(.1, 0, 0);
    }

    public void run(double dir) {
        pidController.setSetpoint(pidController.getSetpoint() + dir * 0.01);
    }

    public void setSetpoint(double setpoint) {
        pidController.setSetpoint(setpoint);
    }

    @Override
    public void periodic() {
        spark.set(pidController.calculate(spark.getEncoder().getPosition()));
    }
}
