package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.model.AlgeePivotLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.utilities.SparkFactory;
import org.littletonrobotics.junction.inputs.LoggableInputs;


public class IntakePivotSubsystem extends SubsystemBase {
    private SparkMax spark;

    private PIDController pidController;

    public IntakePivotSubsystem() {
	spark = SparkFactory.createSparkMax(17);

	
	spark.configure(new SparkMaxConfig().apply(new SoftLimitConfig()
					    .forwardSoftLimit(10)
					    .forwardSoftLimitEnabled(true)
					    .reverseSoftLimit(-10)
					    .reverseSoftLimitEnabled(true)), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

	pidController = new PIDController(.1, 0, 0);
    }

    public void setSetpoint(double setpoint) {
	pidController.setSetpoint(setpoint);
    }
    
    @Override
    public void periodic() {
	spark.set(pidController.calculate(spark.getEncoder().getPosition()));
    }
}
