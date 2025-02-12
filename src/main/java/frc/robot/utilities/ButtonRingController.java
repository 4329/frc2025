package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.model.ButtonRingLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.subsystems.light.LEDState;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class ButtonRingController extends CommandGenericHID implements LoggedSubsystem, Sendable {
  private int level;

  private double xOffset;
  private int button;
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

              LEDState.reefLevel = level;
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
                    button = why;

                    xOffset = why % 2 == 0 ? OFFSET_AMOUNT : -OFFSET_AMOUNT;
                    tagID = AprilTagUtil.getReef((why % 12) / 2);

                    LEDState.reefButton = why;
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

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Octagon");
    builder.addIntegerProperty("button", () -> button, (a) -> button = (int)a);
    builder.addIntegerProperty("level", this::getLevel, (a) -> level = (int)a);
  }
}
