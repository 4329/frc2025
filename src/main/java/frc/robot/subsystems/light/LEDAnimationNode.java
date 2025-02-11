package frc.robot.subsystems.light;

import edu.wpi.first.wpilibj.LEDPattern;

public record LEDAnimationNode(LEDPattern head, YesList nextNodes, String name) {}
