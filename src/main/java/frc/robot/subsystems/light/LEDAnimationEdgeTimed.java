package frc.robot.subsystems.light;

import java.util.function.*;

import edu.wpi.first.wpilibj.Timer;

public class LEDAnimationEdgeTimed implements LEDAnimationEdge {
    LEDAnimationNode node;
    Timer timer;
    double time;

    public LEDAnimationEdgeTimed(LEDAnimationNode node, double timeSeconds) {
        timer = new Timer();
        time = timeSeconds;
    }

    @Override
    public void enter() {
        timer.restart();
    }

    @Override
    public LEDAnimationNode node() {
        return node();
    }

    @Override
    public Supplier<Boolean> transfer() {
        return () -> timer.hasElapsed(time);
    }
}
