package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.model.ButtonRingLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ButtonRingController extends CommandGenericHID implements LoggedSubsystem {
  private double xOffset;
  private int level;
  private int tagID;

  ButtonRingLogAutoLogged buttonRingLogAutoLogged;

  private final double OFFSET_AMOUNT = 0.1651;

  public ButtonRingController(int port) {
    super(port);

    new UnInstantCommand(
            () -> {
              if (getRawAxis(0) == 1) level = 3;
              else if (getRawAxis(0) == -1) level = 4;
              else if (getRawAxis(1) == 1) level = 2;
              else if (getRawAxis(1) == -1) level = 1;
            })
        .repeatedly()
        .ignoringDisable(true)
        .schedule();

    for (int i = 1; i <= 12; i++) {
      final int why = i;
      button(i)
          .onTrue(
              new UnInstantCommand(
                  () -> {
                    xOffset = why % 2 == 0 ? OFFSET_AMOUNT : -OFFSET_AMOUNT;
                    tagID = AprilTagUtil.getReef((why % 12) / 2);
                  }));
    }

    buttonRingLogAutoLogged = new ButtonRingLogAutoLogged();
  }

  @Override
  public LoggableInputs log() {
    buttonRingLogAutoLogged.level = level;
    buttonRingLogAutoLogged.xOffset = xOffset;
    buttonRingLogAutoLogged.tagID = tagID;
    return buttonRingLogAutoLogged;
  }

  public double getxOffset() {
    return xOffset;
  }

  public int getLevel() {
    return level;
  }

  public int getTagID() {
    return tagID;
  }
}
