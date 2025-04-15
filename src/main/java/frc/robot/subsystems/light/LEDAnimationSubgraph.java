package frc.robot.subsystems.light;

import java.util.List;
import java.util.function.*;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj.LEDPattern;

public class LEDAnimationSubgraph implements LEDAnimationNode {

    LEDAnimationNode current;
    List<LEDAnimationEdge> nextNodes;
    String name;
    LEDAnimationNode children;

    public LEDAnimationSubgraph(LEDAnimationNode children, List<LEDAnimationEdge> nextNodes, String name) {
        this.name = name;
        this.nextNodes = nextNodes;

        this.current = children;
        this.children = children;
        current.nextNodes().forEach(x -> x.enter());
    }

    public LEDPattern animation() {
        return current.animation();
    }

    public List<LEDAnimationEdge> nextNodes() {
        return nextNodes;
    }

    public String name() {
        return name;
    }

    @Override
    public boolean equals(Object other) {
        if (other == null || !(other instanceof LEDAnimationSubgraph)) return false;

        LEDAnimationSubgraph otro = (LEDAnimationSubgraph) other;
        return otro.current.equals(this.current)
                && otro.nextNodes.equals(this.nextNodes)
                && otro.name.equals(this.name)
                && otro.children.equals(this.children);
    }

    @Override
    public int hashCode() {
        return current.hashCode() + name.hashCode() + children.hashCode();
    }

    @Override
    public void exit() {
        current = children;
    }

    public void check() {
        if (current instanceof LEDAnimationSubgraph) ((LEDAnimationSubgraph)current).check();
        current
            .nextNodes()
            .forEach(
                    x -> {
                        if (x.transfer().get()) {
                            current.exit();
                            current = x.node();
                            x.node().nextNodes().forEach(y -> y.enter());
                        }
                    });

    }

    public boolean over() {
        return current == null;
    }

    public String log() {
        return name + "(" + current.log() + ")";
    }

    @Override
    public void add(LEDAnimationNode node, Supplier<Boolean> transfer) {
        nextNodes().add(new LEDAnimationEdgeSimple(node, transfer));
    }
}
