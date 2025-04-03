package frc.robot.subsystems.lilih.climber;

import static org.mockito.Mockito.mockStatic;
import static org.mockito.Mockito.when;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.utilities.SparkFactory;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.MockedStatic;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
public class ClimberSubsystemTest {
    @Mock SparkMax mockMotor;
    @Mock RelativeEncoder mockEncoder;
    ClimberSubsystem climberSubsystem;

    @BeforeEach
    public void init() {
        try (MockedStatic<SparkFactory> mockedStatic = mockStatic(SparkFactory.class)) {
            mockedStatic
                    .when(() -> SparkFactory.createSparkMax(Constants.SparkIDs.climber))
                    .thenReturn(mockMotor);
            when(mockMotor.getEncoder()).thenReturn(mockEncoder);
        }
    }
}
