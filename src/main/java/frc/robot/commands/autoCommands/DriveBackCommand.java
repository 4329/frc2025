package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;

public class DriveBackCommand extends Command {
    Drivetrain drivetrain;
    Timer timer;

    public DriveBackCommand(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        timer = new Timer();
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public void execute() {
        drivetrain.drive(0, -0.25, 0, false);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.5);
    }
}
