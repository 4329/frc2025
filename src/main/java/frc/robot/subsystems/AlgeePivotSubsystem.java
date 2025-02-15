package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SparkFactory;

public class AlgeePivotSubsystem extends SubsystemBase {
    SparkMax motor;
    SparkBaseConfig config;
    SparkClosedLoopController sparkClosedLoopController;

    PIDController pidController;

    public AlgeePivotSubsystem() {
        motor = SparkFactory.createSparkMax(9);
        config =
                new SparkMaxConfig()
                        .apply(
                                new SoftLimitConfig()
                                        .forwardSoftLimit(10)
                                        .forwardSoftLimitEnabled(true)
                                        .reverseSoftLimit(-10)
                                        .reverseSoftLimitEnabled(true));
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        sparkClosedLoopController = motor.getClosedLoopController();
        pidController = new PIDController(.1, 0, 0);
    }

    public void setSetpoint(double setpoint) {
        pidController.setSetpoint(setpoint);
    }

    public void run(double speed) {
        setSetpoint(motor.getEncoder().getPosition() + speed);
    }

    @Override
    public void periodic() {
        motor.set(pidController.calculate(motor.getEncoder().getPosition()));
    }
}
