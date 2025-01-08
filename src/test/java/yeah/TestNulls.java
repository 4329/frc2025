package yeah;

import static org.junit.jupiter.api.Assertions.fail;

import frc.robot.Robot;
import org.junit.jupiter.api.Test;

public class TestNulls {
  @Test
  void checkBuilds() {
    try {
      Robot robot = new Robot();
      robot.robotInit();
    } catch (Exception e) {
      System.out.println(e.getStackTrace());
      fail("Didn't Build");
    }
  }
}
