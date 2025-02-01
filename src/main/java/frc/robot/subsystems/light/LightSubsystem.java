package frc.robot.subsystems.light;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.model.LightLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LightSubsystem extends SubsystemBase implements LoggedSubsystem {
  AddressableLED addressableLED;
  AddressableLEDBuffer addressableLEDBuffer;

  private LEDAnimationNode currentAnimation;

  private LightLogAutoLogged lightLogAutoLogged;

  public LightSubsystem() {
    addressableLED = new AddressableLED(0);
    addressableLEDBuffer = new AddressableLEDBuffer(60);

    addressableLED.setLength(addressableLEDBuffer.getLength());
    addressableLED.setData(addressableLEDBuffer);
    addressableLED.start();

    LEDPattern teal = LEDPattern.solid(Color.kTeal);
    teal.applyTo(addressableLEDBuffer);
    addressableLED.setData(addressableLEDBuffer);

    currentAnimation = createGraph();
  
    lightLogAutoLogged = new LightLogAutoLogged();
  }

  private LEDAnimationNode createGraph() {
    LEDAnimationNode head = new LEDAnimationNode(LEDPattern.solid(Color.kRed), () -> !LEDState.on, new YesList(), "head");
    LEDAnimationNode orange =
        new LEDAnimationNode(
            LEDPattern.rainbow(255, 128)
                .scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meter.of(1.0 / 120.0)),
            () -> LEDState.on,
            new YesList(),
            "orange");

    head.nextNodes().add(orange);
    orange.nextNodes().add(head);

    return head;
  }

  private void resolveGraph() {
    currentAnimation.nextNodes().forEach(
        x -> {
          if (x.transfer().get()) {
            currentAnimation = x;
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
    lightLogAutoLogged.name = currentAnimation.name();
    lightLogAutoLogged.state = LEDState.asString();

    return lightLogAutoLogged;
  }
}
