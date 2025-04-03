package frc.robot.commands.climberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem.ClimberPosition;

public class ClimberPivotCommand extends Command {
    private ClimberSubsystem climberSubsystem;
    private ClimberPosition climby;

    public ClimberPivotCommand(ClimberSubsystem climberSubsystem, ClimberPosition climby) {
        this.climberSubsystem = climberSubsystem;
        this.climby = climby;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.setSetpoint(climby);
    }

    @Override
    public boolean isFinished() {
        return climberSubsystem.atSetpoint();
    }
}
