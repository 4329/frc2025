package frc.robot.commands.elevatorCommands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import org.littletonrobotics.junction.Logger;

public class CoolEvator extends Command {

    GenericEntry yep = Shuffleboard.getTab("Asdf").add("elesetpoint", 0).getEntry();
    double oldYep;
    ElevatorSubsystem elevatorSubsystem;

    public CoolEvator(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        oldYep = yep.getDouble(0);
        new SetElevatorCommand(
                        elevatorSubsystem, ElevatorPosition.values()[(int) (yep.getDouble(0) * 2)])
                .schedule();
    }

    @Override
    public void execute() {
        Logger.recordOutput("asdf", yep.getDouble(0));
    }

    @Override
    public void end(boolean interrupted) {
        new SetElevatorCommand(
                        elevatorSubsystem, ElevatorPosition.values()[(int) (yep.getDouble(0) * 2 + 1)])
                .schedule();
    }

    @Override
    public boolean isFinished() {
        double newYep = yep.getDouble(0);

        boolean answer = oldYep != newYep;
        oldYep = newYep;

        return answer;
    }
}
