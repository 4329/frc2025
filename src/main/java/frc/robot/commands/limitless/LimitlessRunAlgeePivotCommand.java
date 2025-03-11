package frc.robot.commands.limitless;

import frc.robot.commands.algeePivotCommands.RunAlgeePivotCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;

public class LimitlessRunAlgeePivotCommand extends RunAlgeePivotCommand {
    
    public LimitlessRunAlgeePivotCommand(AlgeePivotSubsystem algeePivotSubsystem, double speed) {
        super(algeePivotSubsystem, speed);
    }

    @Override
    public void initialize() {
        algeePivotSubsystem.enableLimits(false);
    }

    @Override
    public void end(boolean interrupted) {
        algeePivotSubsystem.enableLimits(true);
        super.end(interrupted);
    }
}
