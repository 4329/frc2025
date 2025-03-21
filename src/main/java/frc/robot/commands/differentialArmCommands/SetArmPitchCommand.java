package frc.robot.commands.differentialArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem.DifferentialArmPitch;

public class SetArmPitchCommand extends Command {
    private final DifferentialArmSubsystem differentialArmSubsystem;
    private final DifferentialArmSubsystem.DifferentialArmPitch pitchTarget;

    public SetArmPitchCommand(
            DifferentialArmSubsystem differentialArmSubsystem, DifferentialArmPitch pitchTarget) {
        this.differentialArmSubsystem = differentialArmSubsystem;
        this.pitchTarget = pitchTarget;
    }

    @Override
    public void execute() {
        differentialArmSubsystem.setPitchTarget(pitchTarget);
    }

    @Override
    public boolean isFinished() {
        return differentialArmSubsystem.pitchAtSetpoint();
    }
}
