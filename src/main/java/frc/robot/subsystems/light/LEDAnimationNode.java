package frc.robot.subsystems.light;

import edu.wpi.first.wpilibj.LEDPattern;
import java.util.function.Supplier;

public record LEDAnimationNode(
    LEDPattern head, YesList nextNodes, String name) {}