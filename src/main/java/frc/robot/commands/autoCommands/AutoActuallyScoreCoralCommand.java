package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem.DifferentialArmPitch;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.light.LEDState;

public class AutoActuallyScoreCoralCommand extends Command {

    ElevatorSubsystem elevatorSubsystem;
    DifferentialArmSubsystem differentialArmSubsystem;
    ElevatorPosition elevatorPosition;
    Timer timer;

    public AutoActuallyScoreCoralCommand(
            ElevatorSubsystem elevatorSubsystem,
            DifferentialArmSubsystem differentialArmSubsystem,
            ElevatorPosition elevatorPosition) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.differentialArmSubsystem = differentialArmSubsystem;
        this.elevatorPosition = elevatorPosition;
    }

    @Override
    public void initialize() {
        LEDState.scoreCoral = true;
        elevatorSubsystem.setSetpoint(elevatorPosition);
        if (ElevatorPosition.L4.equals(elevatorPosition)) {
            differentialArmSubsystem.setPitchTarget(DifferentialArmPitch.NINETY);
        }
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atSetpoint() && differentialArmSubsystem.pitchAtSetpoint();
    }
}
