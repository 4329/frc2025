package frc.robot.subsystems.light;

import edu.wpi.first.wpilibj.LEDPattern;
import java.util.List;

public record LEDAnimationNode(LEDPattern head, List<LEDAnimationEdge> nextNodes, String name) {
    @Override
    public boolean equals(Object other) {
        if (other == null || !(other instanceof LEDAnimationNode)) return false;

        LEDAnimationNode otro = (LEDAnimationNode) other;
        return otro.head.equals(this.head)
                && otro.nextNodes.equals(this.nextNodes)
                && otro.name.equals(this.name);
    }

    @Override
    public int hashCode() {
        return head.hashCode() + name.hashCode();
    }
}
