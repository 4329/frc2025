package frc.robot.subsystems.light;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.model.LightLogEntry;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.subsystems.light.ledAnimations.CoutPattern;
import frc.robot.subsystems.light.ledAnimations.GrowPattern;
import java.util.ArrayList;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LightSubsystem extends SubsystemBase implements LoggedSubsystem {
    AddressableLED addressableLED;
    AddressableLEDBuffer addressableLEDBuffer;

    private LEDAnimationNode currentAnimation;

    private LightLogEntry lightLogEntry;

    public LightSubsystem() {
        addressableLED = new AddressableLED(0);
        addressableLEDBuffer = new AddressableLEDBuffer(128);

        addressableLED.setLength(addressableLEDBuffer.getLength());
        addressableLED.setData(addressableLEDBuffer);
        addressableLED.start();

        LEDPattern teal = LEDPattern.solid(Color.kTeal);
        teal.applyTo(addressableLEDBuffer);
        addressableLED.setData(addressableLEDBuffer);

        currentAnimation = createGraph();

        lightLogEntry = new LightLogEntry();
    }

    private LEDAnimationNode createGraph() {
        LEDAnimationNode idle =
                new LEDAnimationNode(
                        (reader, writer) -> {
                            LEDPattern.rainbow(255, 255)
                                    .scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meter.of(1.0 / 120.0))
                                    .applyTo(reader, writer);
                            writer.setLED((int) (Math.random() * reader.getLength()), Color.kWhite);
                        },
                        new ArrayList<>(),
                        "idle");

        LEDAnimationNode centering =
                new LEDAnimationNode(new CoutPattern(), new ArrayList<>(), "centering");

        LEDAnimationNode targetVisible =
                new LEDAnimationNode(new GrowPattern(), new ArrayList<>(), "targetVisible");

        idle.nextNodes().add(new LEDAnimationEdge(centering, () -> LEDState.centerRunning));
        centering.nextNodes().add(new LEDAnimationEdge(idle, () -> !LEDState.centerRunning));

        idle.nextNodes().add(new LEDAnimationEdge(targetVisible, () -> LEDState.targetVisible));
        targetVisible.nextNodes().add(new LEDAnimationEdge(idle, () -> !LEDState.targetVisible));

        return idle;
    }

    private void resolveGraph() {
        currentAnimation
                .nextNodes()
                .forEach(
                        x -> {
                            if (x.transfer().get()) {
                                currentAnimation = x.node();
                            }
                        });

        currentAnimation.head().applyTo(addressableLEDBuffer);
        addressableLED.setData(addressableLEDBuffer);
    }

    @Override
    public void periodic() {
        resolveGraph();
    }

    @Override
    public LoggableInputs log() {
        lightLogEntry.name = currentAnimation.name();
        return lightLogEntry;
    }
}
