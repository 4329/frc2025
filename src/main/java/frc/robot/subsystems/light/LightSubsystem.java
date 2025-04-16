package frc.robot.subsystems.light;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.model.LightLogEntry;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.subsystems.light.ledAnimations.BeamsPattern;
import frc.robot.subsystems.light.ledAnimations.CoutPattern;
import frc.robot.subsystems.light.ledAnimations.GrowPattern;
import frc.robot.subsystems.light.ledAnimations.PolicePattern;
import frc.robot.subsystems.light.ledAnimations.RisingPattern;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LightSubsystem extends SubsystemBase implements LoggedSubsystem {
    public static int SIDE_LENGTH = 45;

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
        a.add(b, () -> condition.getAsBoolean());
        b.add(a, () -> !condition.getAsBoolean());
    }

    private LEDAnimationNode createGraph() {
        LEDAnimationNodeSimple start = new LEDAnimationNodeSimple(LEDPattern.rainbow(255, 255).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meter.of(1 / 60.0)), new ArrayList<>(), "start") ;
        LEDAnimationNodeSimple goingOut = new LEDAnimationNodeSimple(new BeamsPattern(Color.kOrange, Color.kBlack), new ArrayList<>(), "goingOut");
        LEDAnimationNodeSimple autoMovement = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kYellow), new ArrayList<>(), "autoMovement");
        LEDAnimationNodeSimple autoHping = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kBeige), new ArrayList<>(), "autoHping");
        LEDAnimationNodeSimple rising = new LEDAnimationNodeSimple(new RisingPattern(), new ArrayList<>(), "rising");
        LEDAnimationNodeSimple autoAnticipation = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kOrangeRed), new ArrayList<>(), "autoAnticipation");
        LEDAnimationNodeSimple autoConglaturations = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kPink), new ArrayList<>(), "autoConglaturations");
        LEDAnimationSubgraph beginning = new LEDAnimationSubgraph(start, new ArrayList<>(), "beginning");

        start.add(goingOut, DriverStation::isEnabled);
        goingOut.add(autoMovement, () -> LEDState.out);

        autoMovement.add(rising, () -> !LEDState.elevatorAtSetpoint);
        rising.add(autoAnticipation, () -> LEDState.elevatorAtSetpoint);
        autoConglaturations.nextNodes().add(new LEDAnimationEdgeTimed(autoMovement, 1));
        backForth(autoMovement, autoHping, () -> LEDState.byHpStation);

        LEDAnimationNodeSimple movement = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kGainsboro), new ArrayList<>(), "movement");
        beginning.add(movement, () -> DriverStation.isTeleopEnabled());

        LEDAnimationNodeSimple reefing = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kFirebrick), new ArrayList<>(), "reefing");
        LEDAnimationNodeSimple hping = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kChocolate), new ArrayList<>(), "hping");
        LEDAnimationNodeSimple processoring = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kCadetBlue), new ArrayList<>(), "processoring");
        LEDAnimationNodeSimple parking = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kDarkCyan), new ArrayList<>(), "parking");
        LEDAnimationNodeSimple barging = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kOrchid), new ArrayList<>(), "barging");

        backForth(movement, reefing, () -> LEDState.byReef);
        backForth(movement, hping, () -> LEDState.byHpStation);
        backForth(movement, processoring, () -> LEDState.byPorcessor);
        backForth(movement, parking, () -> LEDState.byBarge && DriverStation.getMatchTime() <= 15);
        backForth(movement, barging, () -> LEDState.byBarge && DriverStation.getMatchTime() > 15);

        LEDAnimationNodeSimple conglaturations = new LEDAnimationNodeSimple(LEDPattern.gradient(GradientType.kContinuous, new Color[] {
            Color.kFirebrick,
            Color.kGainsboro
        }).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meter.of(1.0 / 60)), new ArrayList<>(), "conglaturations");
        conglaturations.nextNodes().add(new LEDAnimationEdgeTimed(movement, 1));
 
        LEDAnimationNodeSimple sad = new LEDAnimationNodeSimple(LEDPattern.gradient(GradientType.kContinuous, new Color[] {
            Color.kDarkBlue,
            Color.kSkyBlue
        }).scrollAtAbsoluteSpeed(MetersPerSecond.of(0.1), Meter.of(1.0 / 60)), new ArrayList<>(), "sad");
        sad.nextNodes().add(new LEDAnimationEdgeTimed(movement, 1));

        
        LEDAnimationNodeSimple coralCentering = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kOrchid), new ArrayList<>(), "coralCentering");
        LEDAnimationNodeSimple flash = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kOrchid), new ArrayList<>(), "flash");
        LEDAnimationNodeSimple coralRising = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kOrchid), new ArrayList<>(), "coralRising");
        LEDAnimationNodeSimple anticipation = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kOrchid), new ArrayList<>(), "anticipation");
        LEDAnimationSubgraph scoring = new LEDAnimationSubgraph(coralCentering, new ArrayList<>(), "scoring");
        movement.add(scoring, () -> LEDState.scoreWithArm);

        coralCentering.add(flash, () -> LEDState.centered);
        flash.nextNodes().add(new LEDAnimationEdgeTimed(coralRising, .25));
        coralRising.add(anticipation, () -> LEDState.elevatorAtSetpoint);
        scoring.add(conglaturations, () -> LEDState.scoreCoral);
        scoring.add(sad, () -> !LEDState.scoreWithArm && !LEDState.scoreCoral);

        LEDAnimationNodeSimple porcessorCentering = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kBisque), new ArrayList<>(), "porcessorCentering");
        LEDAnimationNodeSimple lowering = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kBisque), new ArrayList<>(), "lowering");
        LEDAnimationNodeSimple porcessorAlgeeSpinning = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kBisque), new ArrayList<>(), "porcessorAlgeeSpinning");
        LEDAnimationSubgraph porcessorScoring = new LEDAnimationSubgraph(porcessorCentering, new ArrayList<>(), "porcessorScoring");
        movement.add(porcessorScoring, () -> LEDState.porcessor);

        porcessorCentering.add(lowering, () -> LEDState.centered);
        lowering.add(porcessorAlgeeSpinning, () -> LEDState.elevatorAtSetpoint);
        porcessorScoring.add(conglaturations, () -> !LEDState.porcessor);

        LEDAnimationNodeSimple bargeRising = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kCrimson), new ArrayList<>(), "bargeRising");
        LEDAnimationNodeSimple bargeFlash = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kGold), new ArrayList<>(), "bargeFlash");
        LEDAnimationNodeSimple bargeAnticipation = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kFuchsia), new ArrayList<>(), "bargeAnticipation");
        LEDAnimationSubgraph bargeScoring = new LEDAnimationSubgraph(bargeRising, new ArrayList<>(), "bargeScoring");
        barging.add(bargeScoring, () -> LEDState.elevatorSetpointBarge);

        bargeRising.add(bargeFlash, () -> LEDState.elevatorAtSetpoint);
        bargeFlash.nextNodes().add(new LEDAnimationEdgeTimed(bargeAnticipation, 0.25));
        bargeScoring.add(conglaturations, () -> LEDState.algeeWheelRunning);

        LEDAnimationNodeSimple algeeRising = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kDenim), new ArrayList<>(), "algeeRising");
        LEDAnimationNodeSimple algeeSpinning = new LEDAnimationNodeSimple(LEDPattern.solid(Color.kDenim), new ArrayList<>(), "algeeSpinning");
        LEDAnimationSubgraph algeeIntaking = new LEDAnimationSubgraph(algeeRising, new ArrayList<>(), "algeeIntaking");
        movement.add(algeeIntaking, () -> LEDState.algeeIntaking);

        algeeRising.add(algeeSpinning, () -> LEDState.algeeWheelRunning);
        algeeRising.add(conglaturations, () -> !LEDState.algeeIntaking);

        return new LEDAnimationNodeSimple(new RisingPattern(), new ArrayList<>(), "test");
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
