package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SparkFactory;

public class ElevatorSubsystem extends SubsystemBase {
    SparkMax michael;
    SparkMax jackson;

    public ElevatorSubsystem(){
        michael=SparkFactory.createSparkMax(11);
        jackson=SparkFactory.createSparkMax(12);

    }
    
}
