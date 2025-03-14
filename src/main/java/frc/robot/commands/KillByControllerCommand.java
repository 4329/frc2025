package frc.robot.commands;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.loggedComands.LoggedCommandComposer;

public class KillByControllerCommand extends LoggedCommandComposer implements LoggableInputs {

    XboxController controller;
    Command child;

    boolean finished;

    String childState;

    public KillByControllerCommand(XboxController controller, Command child) {
        setName("Kill(" + child.getName() + ")");
        this.controller = controller;
        this.child = child;
    }

    @Override
    public void initialize() {
        child.initialize();
        childState = "initialize()";
    }

    @Override
    public void execute() {
        if (!finished) {
            child.execute();
            childState = "execute()";
            if (child.isFinished()) {
                finished = true;
                child.end(false);
                childState = "end(false)";
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        child.end(interrupted);
        childState = "end(" + interrupted+ ")";
    }

    @Override
    public boolean isFinished() {
        return controller.getAButtonPressed();
    }

    @Override
    public void toLog(LogTable table) {
        table.put(child.getName(), childState);
    }

    @Override
    public void fromLog(LogTable table) {
        childState = table.get(child.getName()).getString();
    }
}
