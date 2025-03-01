package frc.robot.commands.algeePivotCommands;

import static org.mockito.Mockito.verify;

import frc.robot.subsystems.AlgeePivotSubsystem;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
public class RunAlgeePivotCommandTest {
    @Mock AlgeePivotSubsystem algeePivotSubsystem;
    RunAlgeePivotCommand runAlgeePivotCommand;

    @BeforeEach
    public void init() {
        runAlgeePivotCommand = new RunAlgeePivotCommand(algeePivotSubsystem, 1);
    }

    @Test
    public void testThing() {
        runAlgeePivotCommand.initialize();
        verify(algeePivotSubsystem).run(1);
    }
}
