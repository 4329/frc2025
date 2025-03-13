package frc.robot.commands;

import java.util.Arrays;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.commandGroups.StartCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeeWheelSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.UnInstantCommand;
import frc.robot.utilities.loggedComands.LoggedSequentialCommandGroup;
import frc.robot.commands.elevatorCommands.*;
import frc.robot.commands.commandGroups.HPStationCommand;

public class DoAFunctionalCommand extends LoggedSequentialCommandGroup {
    private final double speed = 0.1;
    private final double rotSpeed = 0.1;

    public DoAFunctionalCommand(Drivetrain drivetrain, XboxController controller, ElevatorSubsystem elevatorSubsystem,
            DifferentialArmSubsystem differentialArmSubsystem, AlgeePivotSubsystem algeePivotSubsystem,
            AlgeeWheelSubsystem algeeWheelSubsystem) {
        Command[] commands = new Command[] {
                new UnInstantCommand(
                        "forward",
                        () -> drivetrain.drive(0, speed, 0, false)),
                new UnInstantCommand(
                        "back",
                        () -> drivetrain.drive(0, -speed, 0, false)),
                new UnInstantCommand(
                        "left",
                        () -> drivetrain.drive(-speed, 0, 0, false)),
                new UnInstantCommand(
                        "right",
                        () -> drivetrain.drive(speed, 0, 0, false)),
                new UnInstantCommand(
                        "rotRight",
                        () -> drivetrain.drive(0, 0, rotSpeed, false)),
                new UnInstantCommand(
                        "rotLeft",
                        () -> drivetrain.drive(0, 0, -rotSpeed, false)),
                new UnInstantCommand(
                        "stop",
                        () -> drivetrain.drive(0, 0, 0, false)),

                new StartCommand(elevatorSubsystem, differentialArmSubsystem, algeePivotSubsystem),

                new UnInstantCommand(
                        "algeein",
                        () -> algeeWheelSubsystem.run(1)),

                new UnInstantCommand(
                        "algeeout",
                        () -> algeeWheelSubsystem.run(-1)),

                new UnInstantCommand(
                        "algeeStop",
                        () -> algeeWheelSubsystem.stop()),

                new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.NET),

                new HPStationCommand(differentialArmSubsystem, elevatorSubsystem, algeePivotSubsystem)


        };

        Command[] outCommands = new Command[commands.length * 3];
        for (int i = 0; i < outCommands.length; i += 3) {
            System.out.println(Arrays.toString(outCommands));
            outCommands[i] = commands[i / 3];
            outCommands[i + 1] = new WaitUntilCommand(controller::getAButtonPressed);
            outCommands[i + 2] = new WaitUntilCommand(controller::getAButtonReleased);
        }

        addCommands(outCommands);
    }
}
