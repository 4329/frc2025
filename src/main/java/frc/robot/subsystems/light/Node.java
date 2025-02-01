package frc.robot.subsystems.light;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.LEDPattern;

public record Node(
        LEDPattern head, Supplier<Boolean> transfer, List nextNodes, String name) { }
