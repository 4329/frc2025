package frc.robot.commands.algeePivotCommands;

import frc.robot.subsystems.AlgeePivotSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class SetAlgeePivotCommand extends Command {
    AlgeePivotSubsystem algeePivotSubsystem;
    


    public SetAlgeePivotCommand(AlgeePivotSubsystem algeePivotSubsystem) {
        this.algeePivotSubsystem = algeePivotSubsystem;
    }



    @Override
    public boolean isFinished() {
        return algeePivotSubsystem.atSetpoint();
    }
}