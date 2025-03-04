package frc.robot.commands.algeePivotCommands;

import static org.mockito.Mockito.verify;

import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
public class SetAlgeePivotCommandTest {
    @Mock AlgeePivotSubsystem algeePivotSubsystem;
    SetAlgeePivotCommand setAlgeePivotCommand;

    @BeforeEach
    public void init() {
        setAlgeePivotCommand = new SetAlgeePivotCommand(algeePivotSubsystem, AlgeePivotAngle.OUT);
    }

    @Test
    public void testExecute() {
        setAlgeePivotCommand.execute();
        verify(algeePivotSubsystem).setSetpoint(AlgeePivotAngle.OUT);
    }
}
