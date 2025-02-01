package frc.robot.subsystems.light;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.LEDPattern;

public record LEDAnimationNode(
        LEDPattern head, Supplier<Boolean> transfer, YesList nextNodes, String name) { }
