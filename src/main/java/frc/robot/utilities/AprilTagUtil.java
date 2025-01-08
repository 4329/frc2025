package frc.robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class AprilTagUtil {

  public static int getAprilTagSpeakerIDAprilTagIDSpeaker() {

    Optional<Alliance> allly = DriverStation.getAlliance();

    if (allly.isPresent() && allly.get().equals(DriverStation.Alliance.Red)) {
      return 4;
    } else {
      return 7;
    }
  }
}
