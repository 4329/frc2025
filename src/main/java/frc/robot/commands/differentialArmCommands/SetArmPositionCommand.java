package frc.robot.commands.differentialArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.differentialArmSubsystem.DifferentialArmSubsystem;

public class SetArmPositionCommand extends Command {
    DifferentialArmSubsystem jupiter;

    double pitchTarget;
    double rollTarget;

    public SetArmPositionCommand(DifferentialArmSubsystem jupiter, double pitchTarget, double rollTarget) {
        this.jupiter = jupiter;
        this.pitchTarget = pitchTarget;
        this.rollTarget = rollTarget;

        addRequirements(jupiter);
    }

    @Override
    public void initialize() {
        jupiter.setPitchTarget(pitchTarget);
        jupiter.setRollTarget(rollTarget);
    }

    @Override
    public boolean isFinished() {
        return false;//jupiter.pitchAtSetpoint() && jupiter.rollAtSetpoint();

    }
}