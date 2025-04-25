package frc.robot.subsystems.light;

import java.util.function.Supplier;

public record LEDAnimationEdgeStart(LEDAnimationNode node, Supplier<Boolean> transfer)
        implements LEDAnimationEdge {

    @Override
    public boolean equals(Object other) {
        if (other == null || !(other instanceof LEDAnimationEdgeStart)) return false;

        LEDAnimationEdgeStart otro = (LEDAnimationEdgeStart) other;
        return otro.node.equals(this.node) && otro.transfer.equals(this.transfer);
    }

    @Override
    public int hashCode() {
        return node.hashCode() + transfer.hashCode();
    }

    public void enter() {}
}
