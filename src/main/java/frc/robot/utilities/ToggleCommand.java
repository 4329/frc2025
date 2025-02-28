package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;

public class ToggleCommand extends Command {

    private boolean on;
    private Command child;

    public ToggleCommand(Command child) {
        this.child = child;
        setName("Toggle(" + child.getName() + ")");
    }

    @Override
    public void initialize() {
        if (on) child.cancel();
        else child.schedule();

        on = !on;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
