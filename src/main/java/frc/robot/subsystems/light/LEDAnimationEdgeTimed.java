package frc.robot.subsystems.light;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.*;

public class LEDAnimationEdgeTimed implements LEDAnimationEdge {
    LEDAnimationNode node;
    Timer timer;
    double time;

    public LEDAnimationEdgeTimed(LEDAnimationNode node, double timeSeconds) {
        timer = new Timer();
        time = timeSeconds;
        this.node = node;
    }

    @Override
    public void enter() {
        timer.restart();
    }

    @Override
    public LEDAnimationNode node() {
        return node;
    }

    @Override
    public Supplier<Boolean> transfer() {
        return () -> timer.hasElapsed(time);
    }
}
