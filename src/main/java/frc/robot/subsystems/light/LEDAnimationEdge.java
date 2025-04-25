package frc.robot.subsystems.light;

import java.util.function.Supplier;

public interface LEDAnimationEdge {
    public LEDAnimationNode node();

    public Supplier<Boolean> transfer();

    public void enter();
}
