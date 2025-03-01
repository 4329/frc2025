package frc.robot.subsystems.lilih;

import static org.junit.Assert.fail;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
public class LilihSubsystemTest {
    LilihSubsystem lilihSubsystem;

    @BeforeEach
    public void init() {
        lilihSubsystem = new LilihSubsystem(0, "");
    }

    @Test
    public void nulls() {
        try {
            lilihSubsystem.getRobotFieldPoseByTag(1);
        } catch (Exception e) {
            fail(e.getMessage());
        }
    }
}
