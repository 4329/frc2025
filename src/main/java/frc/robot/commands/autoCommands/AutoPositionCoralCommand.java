package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem.DifferentialArmPitch;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;

public class AutoPositionCoralCommand extends Command {

    ElevatorSubsystem elevatorSubsystem;
    DifferentialArmSubsystem differentialArmSubsystem;
    ElevatorPosition elevatorPosition;
    AlgeePivotSubsystem algeePivotSubsystem;

    public AutoPositionCoralCommand(
            ElevatorSubsystem elevatorSubsystem,
            DifferentialArmSubsystem differentialArmSubsystem,
            ElevatorPosition elevatorPosition,
            AlgeePivotSubsystem algeePivotSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.differentialArmSubsystem = differentialArmSubsystem;
        this.elevatorPosition = elevatorPosition;
        this.algeePivotSubsystem = algeePivotSubsystem;
    }

    @Override
    public void execute() {
        differentialArmSubsystem.setPitchTarget(
                ElevatorPosition.L4.equals(elevatorPosition)
                        ? DifferentialArmPitch.ONE_THIRTY_FIVE
                        : DifferentialArmPitch.NINETY);
        algeePivotSubsystem.setSetpoint(AlgeePivotAngle.OUTFORCORAL);
        elevatorSubsystem.setSetpoint(elevatorPosition);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atSetpoint() && differentialArmSubsystem.pitchAtSetpoint();
    }
}
