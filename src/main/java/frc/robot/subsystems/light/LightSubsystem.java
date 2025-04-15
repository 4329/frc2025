package frc.robot.subsystems.light;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.model.LightLogEntry;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.subsystems.light.ledAnimations.CoutPattern;
import frc.robot.subsystems.light.ledAnimations.GrowPattern;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LightSubsystem extends SubsystemBase implements LoggedSubsystem {
    AddressableLED addressableLED;
    AddressableLEDBuffer addressableLEDBuffer;

    private LEDAnimationSubgraph graph;

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

        graph = new LEDAnimationSubgraph(createGraph(), new ArrayList<>(), "head");

        lightLogEntry = new LightLogEntry();
    }

    private void backForth(LEDAnimationNode a, LEDAnimationNode b, BooleanSupplier condition) {
        a.nextNodes().add(new LEDAnimationEdge(b, () -> condition.getAsBoolean()));
        b.nextNodes().add(new LEDAnimationEdge(a, () -> !condition.getAsBoolean()));
    }

    private LEDAnimationNode createGraph() {
        LEDAnimationNodeSimple start = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kAqua), new ArrayList<>(), "start") ;
        LEDAnimationNodeSimple goingOut = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kCoral), new ArrayList<>(), "goingOut");
        LEDAnimationNodeSimple autoMovement = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kYellow), new ArrayList<>(), "autoMovement");
        LEDAnimationNodeSimple autoHping = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kBeige), new ArrayList<>(), "autoHping");
        LEDAnimationNodeSimple rising = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kFuchsia), new ArrayList<>(), "rising");
        LEDAnimationNodeSimple autoAnticipation = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kOrangeRed), new ArrayList<>(), "autoAnticipation");
        LEDAnimationNodeSimple autoConglaturations = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kPink), new ArrayList<>(), "autoConglaturations");
        LEDAnimationSubgraph beginning = new LEDAnimationSubgraph(start, new ArrayList<>(), "beginning");

        start.add(goingOut, DriverStation::isEnabled);
        goingOut.add(autoMovement, () -> LEDState.out);
        autoMovement.add(rising, () -> LEDState.elevatorGoingUp);
        rising.add(autoAnticipation, () -> LEDState.elevatorAtSetpoint);
        autoConglaturations.nextNodes().add(new LEDAnimationEdgeTimed(autoMovement, 1));
        backForth(autoMovement, autoHping, () -> LEDState.byHpStation);

        return beginning;
    }

    private void resolveGraph() {
        graph.check();
        graph.current.animation().applyTo(addressableLEDBuffer);
        addressableLED.setData(addressableLEDBuffer);
    }

    @Override
    public void periodic() {
        resolveGraph();
    }

    @Override
    public LoggableInputs log() {
        lightLogEntry.name = graph.log();
        return lightLogEntry;
    }
}
