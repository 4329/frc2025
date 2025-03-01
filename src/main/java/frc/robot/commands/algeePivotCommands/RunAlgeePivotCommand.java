package frc.robot.commands.algeePivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgeePivotSubsystem;

public class RunAlgeePivotCommand extends Command {
    private AlgeePivotSubsystem algeePivotSubsystem;
    private double speed;

    public RunAlgeePivotCommand(AlgeePivotSubsystem algeePivotSubsystem, double speed) {
        this.algeePivotSubsystem = algeePivotSubsystem;
        this.speed = speed;

        addRequirements(algeePivotSubsystem);
    }

    @Override
    public void execute() {
        algeePivotSubsystem.run(speed);
    }

    @Override
    public void end(boolean interrupted) {
        algeePivotSubsystem.run(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
