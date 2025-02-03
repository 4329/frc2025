package frc.robot.subsystems.light;

import java.util.function.Supplier;

public record LEDAnimationEdge(LEDAnimationNode node, Supplier<Boolean> transfer) { }
