// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.loggedComands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;

/**
 * A command that does nothing but takes a specified amount of time to finish.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class LoggedWaitCommand extends LoggedCommandComposer {
    /** The timer used for waiting. */
    protected Timer m_timer = new Timer();

    private final double m_duration;

    /**
     * Creates a new LoggedWaitCommand. This command will do nothing, and end after the specified
     * duration.
     *
     * @param seconds the time to wait, in seconds
     */
    @SuppressWarnings("this-escape")
    public LoggedWaitCommand(double seconds) {
        m_duration = seconds;
        SendableRegistry.setName(this, getName() + ": " + seconds + " seconds");
    }

    /**
     * Creates a new LoggedWaitCommand. This command will do nothing, and end after the specified
     * duration.
     *
     * @param time the time to wait
     */
    public LoggedWaitCommand(Time time) {
        this(time.in(Seconds));
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_duration);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("duration", () -> m_duration, null);
    }
}
