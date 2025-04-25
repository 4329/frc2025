package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem.DifferentialArmPitch;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.light.LEDState;
import frc.robot.utilities.ButtonRingController;

public class ScoreCoralCommand extends Command {

    ElevatorSubsystem elevatorSubsystem;
    DifferentialArmSubsystem differentialArmSubsystem;
    ButtonRingController buttonRingController;
    AlgeePivotSubsystem algeePivotSubsystem;

    public ScoreCoralCommand(
            ElevatorSubsystem elevatorSubsystem,
            DifferentialArmSubsystem differentialArmSubsystem,
            ButtonRingController buttonRingController,
            AlgeePivotSubsystem algeePivotSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.differentialArmSubsystem = differentialArmSubsystem;
        this.buttonRingController = buttonRingController;
        this.algeePivotSubsystem = algeePivotSubsystem;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        LEDState.scoreCoral = true;
        differentialArmSubsystem.setPitchTarget(
                buttonRingController.getLevel() == 2
                        ? DifferentialArmPitch.SCORE_LOW
                        : DifferentialArmPitch.NINETY);

        elevatorSubsystem.setSetpoint(
                switch (buttonRingController.getLevel()) {
                    case 2 -> {
                        algeePivotSubsystem.setSetpoint(AlgeePivotAngle.OUT);
                        yield ElevatorPosition.L2Score;
                    }
                    case 3 -> ElevatorPosition.L3Score;
                    case 4 -> ElevatorPosition.L4;
                    default -> ElevatorPosition.L2;
                });
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atSetpoint() && differentialArmSubsystem.pitchAtSetpoint();
    }
}
