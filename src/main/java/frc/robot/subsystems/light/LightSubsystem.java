package frc.robot.subsystems.light;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.model.LightLogEntry;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.light.ledAnimations.BeamsPattern;
import frc.robot.subsystems.light.ledAnimations.CoutPattern;
import frc.robot.subsystems.light.ledAnimations.FallPattern;
import frc.robot.subsystems.light.ledAnimations.Movement;
import frc.robot.subsystems.light.ledAnimations.RisingPattern;
import frc.robot.subsystems.light.ledAnimations.SparkleBow;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LightSubsystem extends SubsystemBase implements LoggedSubsystem {
    public static int SIDE_LENGTH = 41;

    AddressableLED addressableLED;
    AddressableLEDBuffer addressableLEDBuffer;

    private LEDAnimationSubgraph graph;

    private LightLogEntry lightLogEntry;

    public LightSubsystem() {
        addressableLED = new AddressableLED(0);
        addressableLEDBuffer = new AddressableLEDBuffer(130);

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
        LEDAnimationNodeSimple start =
                new LEDAnimationNodeSimple(new SparkleBow(30), new ArrayList<>(), "start");
        LEDAnimationNodeSimple goingOut =
                new LEDAnimationNodeSimple(
                        new BeamsPattern(Color.kOrange, Color.kBlack), new ArrayList<>(), "goingOut");
        LEDAnimationNodeSimple autoMovement =
                new LEDAnimationNodeSimple(
                        new Movement(Color.kLightYellow), new ArrayList<>(), "autoMovement");

        LEDAnimationNodeSimple autoHsading =
                new LEDAnimationNodeSimple(new Movement(Color.kDodgerBlue), new ArrayList<>(), "hsading");
        LEDAnimationNodeSimple autoEleLow =
                new LEDAnimationNodeSimple(new FallPattern(), new ArrayList<>(), "eleLow");
        backForth(
                autoHsading,
                autoEleLow,
                () ->
                        LEDState.elevatorAtSetpoint
                                && LEDState.elevatorSetpoint == ElevatorPosition.DIFFERENTIAL_ARM_OUT.pos);
        LEDAnimationSubgraph autoHping =
                new LEDAnimationSubgraph(autoHsading, new ArrayList<>(), "hping");

        LEDAnimationNodeSimple rising =
                new LEDAnimationNodeSimple(new RisingPattern(1), new ArrayList<>(), "rising");
        LEDAnimationNodeSimple autoAnticipation =
                new LEDAnimationNodeSimple(
                        LEDPattern.solid(Color.kOrangeRed), new ArrayList<>(), "autoAnticipation");
        LEDAnimationNodeSimple autoConglaturations =
                new LEDAnimationNodeSimple(
                        LEDPattern.solid(Color.kPink), new ArrayList<>(), "autoConglaturations");
        LEDAnimationSubgraph beginning =
                new LEDAnimationSubgraph(start, new ArrayList<>(), "beginning");

        start.add(goingOut, DriverStation::isEnabled);
        goingOut.add(autoMovement, () -> LEDState.out);

        autoMovement.add(
                rising,
                () -> {
                    LEDState.scoreCoral = false;
                    return !LEDState.elevatorAtSetpoint;
                });
        rising.add(autoAnticipation, () -> LEDState.elevatorAtSetpoint);
        autoAnticipation.add(autoConglaturations, () -> LEDState.scoreCoral);
        autoConglaturations.nextNodes().add(new LEDAnimationEdgeTimed(autoMovement, 1));
        backForth(autoMovement, autoHping, () -> LEDState.byHpStation);

        LEDAnimationNodeSimple basicMovement =
                new LEDAnimationNodeSimple(
                        new Movement(Color.kGainsboro), new ArrayList<>(), "basicMovement");
        LEDAnimationNodeSimple reefing =
                new LEDAnimationNodeSimple(new Movement(Color.kFirebrick), new ArrayList<>(), "reefing");

        LEDAnimationNodeSimple hsading =
                new LEDAnimationNodeSimple(new Movement(Color.kDodgerBlue), new ArrayList<>(), "hsading");
        LEDAnimationNodeSimple eleLow =
                new LEDAnimationNodeSimple(new FallPattern(), new ArrayList<>(), "eleLow");
        backForth(
                hsading,
                eleLow,
                () ->
                        LEDState.elevatorAtSetpoint
                                && LEDState.elevatorSetpoint == ElevatorPosition.DIFFERENTIAL_ARM_OUT.pos);
        LEDAnimationSubgraph hping = new LEDAnimationSubgraph(hsading, new ArrayList<>(), "hping");

        LEDAnimationNodeSimple processoring =
                new LEDAnimationNodeSimple(
                        new Movement(Color.kOliveDrab), new ArrayList<>(), "processoring");

        LEDAnimationNodeSimple barging =
                new LEDAnimationNodeSimple(new Movement(Color.kSienna), new ArrayList<>(), "barging");
        LEDAnimationNodeSimple parking =
                new LEDAnimationNodeSimple(new CoutPattern(), new ArrayList<>(), "parking");
        LEDAnimationSubgraph barge = new LEDAnimationSubgraph(barging, new ArrayList<>(), "barge");
        barging.add(parking, () -> DriverStation.getMatchTime() <= 15 || !LEDState.teleoped);

        LEDAnimationSubgraph movement =
                new LEDAnimationSubgraph(basicMovement, new ArrayList<>(), "movement");
        beginning.add(movement, () -> DriverStation.isTeleopEnabled());

        backForth(basicMovement, reefing, () -> LEDState.byReef);
        backForth(basicMovement, hping, () -> LEDState.byHpStation);
        backForth(basicMovement, processoring, () -> LEDState.byPorcessor);
        backForth(basicMovement, barge, () -> LEDState.byBarge);

        LEDAnimationNodeSimple conglaturations =
                new LEDAnimationNodeSimple(
                        LEDPattern.gradient(
                                        GradientType.kContinuous, new Color[] {Color.kFirebrick, Color.kGainsboro})
                                .scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meter.of(1.0 / 60)),
                        new ArrayList<>(),
                        "conglaturations");
        conglaturations.nextNodes().add(new LEDAnimationEdgeTimed(movement, 1));

        LEDAnimationNodeSimple sad =
                new LEDAnimationNodeSimple(
                        LEDPattern.gradient(
                                        GradientType.kContinuous, new Color[] {Color.kDarkBlue, Color.kSkyBlue})
                                .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.1), Meter.of(1.0 / 60)),
                        new ArrayList<>(),
                        "sad");
        sad.nextNodes().add(new LEDAnimationEdgeTimed(movement, 1));

        LEDAnimationNodeSimple coralCentering =
                new LEDAnimationNodeSimple(
                        LEDPattern.solid(Color.kDarkTurquoise), new ArrayList<>(), "coralCentering");
        LEDAnimationNodeSimple flash =
                new LEDAnimationNodeSimple(LEDPattern.solid(Color.kAzure), new ArrayList<>(), "flash");
        LEDAnimationNodeSimple coralRising =
                new LEDAnimationNodeSimple(
                        LEDPattern.solid(Color.kDarkKhaki), new ArrayList<>(), "coralRising");
        LEDAnimationNodeSimple anticipation =
                new LEDAnimationNodeSimple(
                        LEDPattern.solid(Color.kTomato), new ArrayList<>(), "anticipation");
        LEDAnimationSubgraph scoring =
                new LEDAnimationSubgraph(coralCentering, new ArrayList<>(), "scoring");
        movement.add(scoring, () -> LEDState.scoreWithArm);

        coralCentering.add(flash, () -> LEDState.centered);
        flash.nextNodes().add(new LEDAnimationEdgeTimed(coralRising, 0.25));
        coralRising.add(anticipation, () -> LEDState.elevatorAtSetpoint);
        scoring.add(conglaturations, () -> LEDState.scoreCoral);
        scoring.add(sad, () -> !LEDState.scoreWithArm && !LEDState.scoreCoral);

        LEDAnimationNodeSimple porcessorCentering =
                new LEDAnimationNodeSimple(
                        LEDPattern.solid(Color.kBisque), new ArrayList<>(), "porcessorCentering");
        LEDAnimationNodeSimple lowering =
                new LEDAnimationNodeSimple(new RisingPattern(-1), new ArrayList<>(), "lowering");
        LEDAnimationNodeSimple porcessorAlgeeSpinning =
                new LEDAnimationNodeSimple(
                        LEDPattern.solid(Color.kBlanchedAlmond), new ArrayList<>(), "porcessorAlgeeSpinning");
        LEDAnimationSubgraph porcessorScoring =
                new LEDAnimationSubgraph(porcessorCentering, new ArrayList<>(), "porcessorScoring");
        movement.add(porcessorScoring, () -> LEDState.porcessor);

        LEDAnimationNodeSimple danger =
                new LEDAnimationNodeSimple(
                        LEDPattern.solid(Color.kRed).blink(Seconds.of(0.1), Seconds.of(0.1)),
                        new ArrayList<>(),
                        "danger");
        movement.add(
                danger,
                () ->
                        !LEDState.out
                                && Arrays.asList(
                                                ElevatorPosition.L2.pos,
                                                ElevatorPosition.L3.pos,
                                                ElevatorPosition.L4.pos,
                                                ElevatorPosition.NET.pos)
                                        .contains(LEDState.elevatorSetpoint));
        danger.nextNodes().add(new LEDAnimationEdgeTimed(movement, 1));

        porcessorCentering.add(lowering, () -> LEDState.centered);
        lowering.add(porcessorAlgeeSpinning, () -> LEDState.elevatorAtSetpoint);
        porcessorScoring.add(conglaturations, () -> !LEDState.porcessor);

        LEDAnimationNodeSimple bargeRising =
                new LEDAnimationNodeSimple(
                        LEDPattern.solid(Color.kCrimson), new ArrayList<>(), "bargeRising");
        LEDAnimationNodeSimple bargeFlash =
                new LEDAnimationNodeSimple(LEDPattern.solid(Color.kGold), new ArrayList<>(), "bargeFlash");
        LEDAnimationNodeSimple bargeAnticipation =
                new LEDAnimationNodeSimple(
                        LEDPattern.solid(Color.kFuchsia), new ArrayList<>(), "bargeAnticipation");
        LEDAnimationSubgraph bargeScoring =
                new LEDAnimationSubgraph(bargeRising, new ArrayList<>(), "bargeScoring");
        movement.add(bargeScoring, () -> LEDState.elevatorSetpoint == ElevatorPosition.NET.pos);

        bargeRising.add(bargeFlash, () -> LEDState.elevatorAtSetpoint);
        bargeFlash.nextNodes().add(new LEDAnimationEdgeTimed(bargeAnticipation, 0.25));
        bargeScoring.add(sad, () -> LEDState.elevatorSetpoint != ElevatorPosition.NET.pos);
        bargeScoring.add(conglaturations, () -> LEDState.algeeWheelRunning);

        LEDAnimationNodeSimple algeeRising =
                new LEDAnimationNodeSimple(
                        LEDPattern.solid(Color.kDenim), new ArrayList<>(), "algeeRising");
        LEDAnimationNodeSimple algeeSpinning =
                new LEDAnimationNodeSimple(
                        LEDPattern.solid(Color.kDarkSalmon), new ArrayList<>(), "algeeSpinning");
        LEDAnimationSubgraph algeeIntaking =
                new LEDAnimationSubgraph(algeeRising, new ArrayList<>(), "algeeIntaking");
        movement.add(algeeIntaking, () -> LEDState.algeeIntaking);

        algeeRising.add(algeeSpinning, () -> LEDState.algeeWheelRunning);
        algeeIntaking.add(conglaturations, () -> !LEDState.algeeIntaking);

        return beginning;
    }

    private void resolveGraph() {
        graph.check();
        graph.current.animation().applyTo(addressableLEDBuffer);
        addressableLED.setData(addressableLEDBuffer);
    }

    @Override
    public void periodic() {
        if (Timer.getMatchTime() > 100) LEDState.teleoped = true;
        resolveGraph();
    }

    @Override
    public LoggableInputs log() {
        lightLogEntry.name = graph.log();
        return lightLogEntry;
    }
}
