package frc.robot.commands.algeeWheelCommands;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mockStatic;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.AlgeeWheelSubsystem;
import frc.robot.subsystems.light.LEDState;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.MockedStatic;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
public class IntakeAlgeeCommandTest {
    @Mock AlgeeWheelSubsystem algeeWheelSubsystem;
    @Mock CommandScheduler mockCommandScheduler;
    IntakeAlgeeCommand intakeAlgeeCommand;

    @BeforeEach
    public void init() {
        intakeAlgeeCommand = new IntakeAlgeeCommand(algeeWheelSubsystem, 1);
        LEDState.algeeWheelHolding = false;
    }

    @Test
    public void initialize() {
        intakeAlgeeCommand.initialize();
        verify(algeeWheelSubsystem).run(1);
    }
    /*
    // !!!!!!This is an example of mocking static methods in a test!!!!
    @Test
    public void execute_algeed() {
        try (MockedStatic<CommandScheduler> staticCommandScheduler =
                mockStatic(CommandScheduler.class)) {
            staticCommandScheduler.when(CommandScheduler::getInstance).thenReturn(mockCommandScheduler);
            when(algeeWheelSubsystem.getAlgeed()).thenReturn(true);
            intakeAlgeeCommand.execute();
            verify(mockCommandScheduler).cancel(intakeAlgeeCommand);
        }
    }

    @Test
    public void execute_notalgeed() {
        try (MockedStatic<CommandScheduler> staticCommandScheduler =
                mockStatic(CommandScheduler.class)) {
            staticCommandScheduler.when(CommandScheduler::getInstance).thenReturn(mockCommandScheduler);
            when(algeeWheelSubsystem.getAlgeed()).thenReturn(false);
            intakeAlgeeCommand.execute();
            verify(mockCommandScheduler, never()).cancel(any(IntakeAlgeeCommand.class));
        }
    }
    */

    @Test
    public void end_interrupted() {
        intakeAlgeeCommand.end(true);
        assertFalse(LEDState.algeeWheelHolding);
        verify(algeeWheelSubsystem).stop();
    }

    @Test
    public void end_unInterrupted() {
        intakeAlgeeCommand.end(false);
        assertTrue(LEDState.algeeWheelHolding);
        verify(algeeWheelSubsystem).stop();
    }

    @Test
    public void isFinished() {
        when(algeeWheelSubsystem.getAlgeed()).thenReturn(true);
        boolean result = intakeAlgeeCommand.isFinished();
        assertTrue(result);
        verify(algeeWheelSubsystem).getAlgeed();
    }
}
