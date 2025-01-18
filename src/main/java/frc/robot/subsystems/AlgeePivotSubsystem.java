package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SparkFactory;

public class AlgeePivotSubsystem extends SubsystemBase {
    SparkMax motor1;
    PIDController matthew;

    public AlgeePivotSubsystem(){
        motor1=SparkFactory.createSparkMax(15);
    }
    
}


