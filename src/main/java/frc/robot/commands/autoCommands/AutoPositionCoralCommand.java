package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem.DifferentialArmPitch;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.utilities.ButtonRingController;

public class AutoPositionCoralCommand extends Command {

    ElevatorSubsystem elevatorSubsystem;
    DifferentialArmSubsystem differentialArmSubsystem;
    ElevatorPosition elevatorPosition;

    public AutoPositionCoralCommand(
            ElevatorSubsystem elevatorSubsystem,
            DifferentialArmSubsystem differentialArmSubsystem,
            ElevatorPosition elevatorPosition) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.differentialArmSubsystem = differentialArmSubsystem;
        this.elevatorPosition = elevatorPosition;
    }

    @Override
    public void execute() {
        differentialArmSubsystem.setPitchTarget(
                ElevatorPosition.L4.equals(elevatorPosition)
                        ? DifferentialArmPitch.ONETHIRTYFIVE
                        : DifferentialArmPitch.NINETY);

        elevatorSubsystem.setSetpoint(elevatorPosition);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atSetpoint() && differentialArmSubsystem.pitchAtSetpoint();
    }
}
