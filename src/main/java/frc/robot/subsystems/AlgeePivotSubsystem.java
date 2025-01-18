package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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
    
    public AlgeePivotSubsystem(){
        motor = SparkFactory.createSparkMax(15);
        config = new SparkMaxConfig().apply(
            new SoftLimitConfig()
                .forwardSoftLimit(-100).forwardSoftLimitEnabled(true)
                .reverseSoftLimit(100).reverseSoftLimitEnabled(true));
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        sparkClosedLoopController = motor.getClosedLoopController();
    }

    public void setSetpoint(double setpoint) {
        sparkClosedLoopController.setReference(setpoint, ControlType.kMAXMotionPositionControl);
    }
}


