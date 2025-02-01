package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class LightSubsystem extends SubsystemBase {
  AddressableLED addressableLED;
  AddressableLEDBuffer addressableLEDBuffer;

  public static class State {
    public static boolean on;

    public static String asString() {
      return "on:" + on;
    }
  }

  private Node current;

  class List {
    ListNode head;

    class ListNode {
      public Node n;
      public ListNode next;

      public ListNode(Node n, ListNode next) {
        this.n = n;
        this.next = next;
      }
    }

    public void add(Node n) {
      if (head == null) {
        head = new ListNode(n, null);
        return;
      }

      ListNode current = head;
      while (current.next != null) {
        current = current.next;
      }
      current.next = new ListNode(n, null);
    }

    public Node get(int num) {
      ListNode current = head;
      for (int i = 0; i < num && current != null; i++) {
        current = current.next;
      }

      return current.n;
    }

    public void forEach(Consumer<Node> fn) {
      ListNode current = head;

      while (current != null) {
        fn.accept(current.n);
        current = current.next;
      }
    }
  }

  public static record Node(
      LEDPattern head, Supplier<Boolean> transfer, List nextNodes, String name) {
    @Override
    public String toString() {
      return name;
    }
  }

  public LightSubsystem() {
    addressableLED = new AddressableLED(0);
    addressableLEDBuffer = new AddressableLEDBuffer(60);

    addressableLED.setLength(addressableLEDBuffer.getLength());
    addressableLED.setData(addressableLEDBuffer);
    addressableLED.start();

    LEDPattern teal = LEDPattern.solid(Color.kTeal);
    teal.applyTo(addressableLEDBuffer);
    addressableLED.setData(addressableLEDBuffer);

    current = createGraph();
  }

  private Node createGraph() {
    Node head = new Node(LEDPattern.solid(Color.kRed), () -> !State.on, new List(), "head");
    Node orange =
        new Node(
            LEDPattern.rainbow(255, 128)
                .scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meter.of(1.0 / 120.0)),
            () -> State.on,
            new List(),
            "orange");

    head.nextNodes.add(orange);
    orange.nextNodes.add(head);

    return head;
  }

  private void resolveGraph() {
    current.nextNodes.forEach(
        x -> {
          if (x.transfer().get()) {
            current = x;
          }
        });

    current.head.applyTo(addressableLEDBuffer);
    addressableLED.setData(addressableLEDBuffer);
  }

  @Override
  public void periodic() {
    resolveGraph();

    Logger.recordOutput("current", current.name());
    Logger.recordOutput("state", State.asString());
  }
}
