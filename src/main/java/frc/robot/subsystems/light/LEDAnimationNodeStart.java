package frc.robot.subsystems.light;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.LEDPattern;

public record LEDAnimationNodeStart(LEDPattern animation, List<LEDAnimationEdge> nextNodes, String name) implements LEDAnimationNode {

    @Override
    public boolean equals(Object other) {
        if (other == null || !(other instanceof LEDAnimationNodeStart)) return false;

        LEDAnimationNodeStart otro = (LEDAnimationNodeStart) other;
        return otro.animation.equals(this.animation)
                && otro.nextNodes.equals(this.nextNodes)
                && otro.name.equals(this.name);
    }

    @Override
    public int hashCode() {
        return animation.hashCode() + name.hashCode();
    }

    @Override
    public void exit() {}

    @Override
    public String log() {
        return name;
    }

    @Override
    public void add(LEDAnimationNode node, Supplier<Boolean> transfer) {
        nextNodes().add(new LEDAnimationEdgeSimple(node, transfer));
    }
}
