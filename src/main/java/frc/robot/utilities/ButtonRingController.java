package frc.robot.utilities;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.model.ButtonRingLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.subsystems.light.LEDState;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ButtonRingController extends CommandGenericHID implements LoggedSubsystem, Sendable {
    private int level;

    private double xOffset;
    private int button;
    private int tagID;
    private final double OFFSET_AMOUNT = 0.1651;

    ButtonRingLogAutoLogged buttonRingLogAutoLogged;

    public ButtonRingController(int port) {
        super(port);

        new UnInstantCommand(
                        "SetButtonRingLevel",
                        () -> {
                            if (getRawAxis(0) == 1) level = 2;
                            else if (getRawAxis(0) == -1) level = 1;
                            else if (getRawAxis(1) == 1) level = 3;
                            else if (getRawAxis(1) == -1) level = 4;
                            else level = 0;

                            LEDState.reefLevel = level;
                        })
                .repeatedlyLog()
                .ignoringDisableLog(true)
                .schedule();

        for (int i = 1; i <= 12; i++) {
            final int why = i;
            button(i)
                    .onTrue(
                            new UnInstantCommand(
                                            "SetButtonRingButtonDown",
                                            () -> {
                                                button = why;

                                                xOffset = OFFSET_AMOUNT * (why % 2 == 0 ? 1 : -1);
                                                tagID = AprilTagUtil.getReef((why % 12) / 2);

                                                LEDState.reefButton = why;
                                            })
                                    .ignoringDisableLog(true));
            button(i)
                    .onFalse(
                            new UnInstantCommand(
                                            "SetButtonRingButtonDown",
                                            () -> {
                                                if (button == why) {
                                                    button = -1;
                                                    xOffset = 0;
                                                    tagID = 0;
                                                }
                                            })
                                    .ignoringDisableLog(true));
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
        builder.addIntegerProperty("button", () -> button, (a) -> button = (int) a);
        builder.addIntegerProperty("level", this::getLevel, (a) -> level = (int) a);
    }
}
