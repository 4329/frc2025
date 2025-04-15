package frc.robot.subsystems.light;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj.LEDPattern;

public interface LEDAnimationNode {
    public LEDPattern animation();
    public List<LEDAnimationEdge> nextNodes();
    public String name();
    public default void enter() {
        nextNodes().forEach(x -> x.enter());
    }
    public void exit();
    public String log();
    public void add(LEDAnimationNode node, Supplier<Boolean> transfer);
}
