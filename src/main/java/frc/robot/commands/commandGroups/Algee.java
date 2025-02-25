package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AlgeeWheelSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.utilities.ButtonRingController;
import frc.robot.utilities.UnInstantCommand;

    public class Algee {

     public static Command Algee(ElevatorSubsystem elevatorSubsystem, AlgeeWheelSubsystem algeeWheelSubsystem, AlgeePivotSubsystem algeePivotSubsystem, ButtonRingController buttonRingController) {
         return new SequentialCommandGroup(
             new ParallelCommandGroup(
                 new UnInstantCommand(() -> elevatorSubsystem.setSetpoint(buttonRingController.getxOffset() < 0 ? 
                            ElevatorSubsystem.ElevatorPosition.ALGEE_HIGH :
                            ElevatorSubsystem.ElevatorPosition.ALGEE_LOW))
             )
         );
     }
 }