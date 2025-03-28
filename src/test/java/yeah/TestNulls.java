package yeah;

import static org.junit.jupiter.api.Assertions.fail;

import org.junit.jupiter.api.Test;

import frc.robot.Robot;

public class TestNulls {
    @Test
    void checkBuilds() {
       try {
           Robot robot = new Robot();
           robot.robotInit();
           robot.close();
       } catch (Exception e) {
           e.printStackTrace();
           fail("Didn't Build");
       }
    }
}
