package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.model.AlgeePivotLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.utilities.SparkFactory;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class AlgeePivotSubsystem extends SubsystemBase implements LoggedSubsystem {
    private final double ALGEE_PIVOT_SPEED = 0.1;

    private SparkMax motor;
    private SparkBaseConfig config;

    private PIDController pidController;

    public enum AlgeePivotAngle {
        ZERO(0),
        HIGH(Math.PI / 4),
        LOW(-Math.PI / 4);

        public double angle;

        AlgeePivotAngle(double angle) {
            this.angle = angle;
        }
    }

    private final AlgeePivotLogAutoLogged algeePivotLogAutoLogged;

    public AlgeePivotSubsystem() {
        motor = SparkFactory.createSparkMax(100);
        config =
                new SparkMaxConfig()
                        .apply(
                                new SoftLimitConfig()
                                        .forwardSoftLimit(10)
                                        .forwardSoftLimitEnabled(true)
                                        .reverseSoftLimit(-10)
                                        .reverseSoftLimitEnabled(true));
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        pidController = new PIDController(.1, 0, 0);

        algeePivotLogAutoLogged = new AlgeePivotLogAutoLogged();
    }

    private void setSetpoint(double setpoint) {
        pidController.setSetpoint(setpoint);
    }

    public void run(double speed) {
        setSetpoint(pidController.getSetpoint() + speed * ALGEE_PIVOT_SPEED);
    }

    public void setSetpoint(AlgeePivotAngle angle) {
        setSetpoint(angle.angle);
    }

    @Override
    public void periodic() {
        motor.set(pidController.calculate(motor.getEncoder().getPosition()));
    }

    @Override
    public LoggableInputs log() {
        algeePivotLogAutoLogged.setpoint = pidController.getSetpoint();
        return algeePivotLogAutoLogged;
    }
}
