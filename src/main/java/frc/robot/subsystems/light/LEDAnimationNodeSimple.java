package frc.robot.subsystems.light;

import edu.wpi.first.wpilibj.LEDPattern;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.LogTable;

public record LEDAnimationNodeSimple(LEDPattern animation, List<LEDAnimationEdge> nextNodes, String name) implements LEDAnimationNode {
    @Override
    public boolean equals(Object other) {
        if (other == null || !(other instanceof LEDAnimationNodeSimple)) return false;

        LEDAnimationNodeSimple otro = (LEDAnimationNodeSimple) other;
        return otro.animation.equals(this.animation)
                && otro.nextNodes.equals(this.nextNodes)
                && otro.name.equals(this.name);
    }

    @Override
    public int hashCode() {
        return animation.hashCode() + name.hashCode();
    }

    @Override
    public void exit() { }

    @Override
    public String log() {
        return name;
    }

    @Override
    public void add(LEDAnimationNode node, Supplier<Boolean> transfer) {
        nextNodes().add(new LEDAnimationEdgeSimple(node, transfer));
    }
}
