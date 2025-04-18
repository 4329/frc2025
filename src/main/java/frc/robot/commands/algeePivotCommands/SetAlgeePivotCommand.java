package frc.robot.commands.algeePivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgeePivotSubsystem;

public class SetAlgeePivotCommand extends Command {
    AlgeePivotSubsystem algeePivotSubsystem;
    AlgeePivotSubsystem.AlgeePivotAngle algeePivotAngle;

    public SetAlgeePivotCommand(
            AlgeePivotSubsystem algeePivotSubsystem,
            AlgeePivotSubsystem.AlgeePivotAngle algeePivotAngle) {
        this.algeePivotSubsystem = algeePivotSubsystem;
        this.algeePivotAngle = algeePivotAngle;
    }

    @Override
    public void execute() {
        algeePivotSubsystem.setSetpoint(algeePivotAngle);
    }

    @Override
    public boolean isFinished() {
        return algeePivotSubsystem.atSetpoint();
    }
}
