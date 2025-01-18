package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SparkFactory;

public class ElevatorSubsystem extends SubsystemBase {
    SparkMax motor1;
    SparkMax motor2;

    SparkClosedLoopController controller;

    public ElevatorSubsystem(){
        motor1 = SparkFactory.createSparkMax(11);
        motor2 = SparkFactory.createSparkMax(12);

        motor1.configure(
            new SparkMaxConfig().apply(new SoftLimitConfig()
                .forwardSoftLimit(100).forwardSoftLimitEnabled(true)
                .reverseSoftLimit(0).reverseSoftLimitEnabled(true)), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motor2.configure(
                new SparkMaxConfig().follow(motor1, true),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

        controller = motor1.getClosedLoopController();
    }

    public void setSetpoint(double setpoint) {
        controller.setReference(setpoint, ControlType.kMAXMotionPositionControl);
    }
    
}
