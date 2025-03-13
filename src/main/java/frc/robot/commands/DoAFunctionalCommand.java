package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.algeeWheelCommands.IntakeAlgeeCommand;
import frc.robot.commands.algeeWheelCommands.OuttakeAlgeeCommand;
import frc.robot.commands.commandGroups.HPStationCommand;
import frc.robot.commands.commandGroups.HappyResetCommand;
import frc.robot.commands.commandGroups.StartCommand;
import frc.robot.commands.elevatorCommands.SetElevatorCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeeWheelSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.UnInstantCommand;
import frc.robot.utilities.loggedComands.LoggedCommandComposer;
import frc.robot.utilities.loggedComands.LoggedRepeatCommand;
import frc.robot.utilities.loggedComands.LoggedSequentialCommandGroup;
import frc.robot.utilities.loggedComands.LoggedWaitUntilCommand;

public class DoAFunctionalCommand extends LoggedSequentialCommandGroup {
    private final double speed = 1;
    private final double rotSpeed = 1;

    public DoAFunctionalCommand(Drivetrain drivetrain, XboxController controller, ElevatorSubsystem elevatorSubsystem,
            DifferentialArmSubsystem differentialArmSubsystem, AlgeePivotSubsystem algeePivotSubsystem,
            AlgeeWheelSubsystem algeeWheelSubsystem) {
        LoggedCommandComposer[] commands = new LoggedCommandComposer[] {
                new LoggedRepeatCommand(new UnInstantCommand(
                        "forward",
                        () -> drivetrain.drive(0, speed, 0, false))),
                new LoggedRepeatCommand(new UnInstantCommand(
                        "back",
                        () -> drivetrain.drive(0, -speed, 0, false))),
                new LoggedRepeatCommand(new UnInstantCommand(
                        "left",
                        () -> drivetrain.drive(-speed, 0, 0, false))),
                new LoggedRepeatCommand(new UnInstantCommand(
                        "right",
                        () -> drivetrain.drive(speed, 0, 0, false))),
                new LoggedRepeatCommand(new UnInstantCommand(
                        "rotRight",
                        () -> drivetrain.drive(0, 0, rotSpeed, false))),
                new LoggedRepeatCommand(new UnInstantCommand(
                        "rotLeft",
                        () -> drivetrain.drive(0, 0, -rotSpeed, false))),
                new LoggedRepeatCommand(new UnInstantCommand(
                        "stop",
                        () -> drivetrain.stop())),

                new StartCommand(elevatorSubsystem, differentialArmSubsystem, algeePivotSubsystem),

                new IntakeAlgeeCommand(algeeWheelSubsystem),
                new OuttakeAlgeeCommand(algeeWheelSubsystem),

                new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.L2),
                new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.L3),
                new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.L4),
                new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.NET),

                new HPStationCommand(differentialArmSubsystem, elevatorSubsystem, algeePivotSubsystem),

                new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.ZERO).andThenLog(new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.DIFFERENTIAL_ARM_OUT)),

                new HappyResetCommand(differentialArmSubsystem, elevatorSubsystem, algeePivotSubsystem),
        };

        for (int i = 0; i < commands.length; i++) {
            commands[i] = commands[i].onlyWhileLog(() -> !controller.getAButtonPressed()).alongWithLog(new LoggedWaitUntilCommand(controller::getAButtonPressed));
        }

        addCommands(commands);
        addRequirements(drivetrain);
    }
}
