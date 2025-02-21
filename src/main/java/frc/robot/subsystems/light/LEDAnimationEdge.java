package frc.robot.subsystems.light;

import java.util.function.Supplier;

public record LEDAnimationEdge(LEDAnimationNode node, Supplier<Boolean> transfer) {
    @Override
    public boolean equals(Object other) {
        if (other == null || !(other instanceof LEDAnimationEdge)) return false;

        LEDAnimationEdge otro = (LEDAnimationEdge) other;
        return otro.node.equals(this.node) && otro.transfer.equals(this.transfer);
    }

    @Override
    public int hashCode() {
        return node.hashCode() + transfer.hashCode();
    }
}
