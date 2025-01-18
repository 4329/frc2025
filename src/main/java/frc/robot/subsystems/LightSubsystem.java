package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LightSubsystem extends SubsystemBase {

  SerialPort serialPort;
  LEDPattern currentPattern;

  public LightSubsystem() {
    int count = 0;
    try {
      serialPort = new SerialPort(9600, Port.kUSB1);
    } catch (Exception e) {
      count++;
    }
    try {
      serialPort = new SerialPort(9600, Port.kUSB2);
    } catch (Exception e) {
      count++;
    }

    setLEDPattern(LEDPattern.NOTHING);
    if (count == 2) Logger.recordOutput("Lights", "no :(");
    else Logger.recordOutput("Lights", "Yes!");
  }

  public enum LEDPattern {
    BLUE,
    MAGENTA,
    GREEN,
    YELLOW,
    RED,
    ORANGE,
    NOTHING,
    ALRED,
    ALBLUE,
    CLIMB
  }

  public void setLEDPattern(LEDPattern lPattern) {
    if (serialPort != null) {
      byte[] bytey = (lPattern.ordinal() + "\n").getBytes();
      serialPort.write(bytey, bytey.length);

      currentPattern = lPattern;
    } else {
      System.out.println("No USB");
    }
    Logger.recordOutput("Current pattern", lPattern);
  }

  public LEDPattern getLEDPattern() {
    return currentPattern;
  }

  @Override
  public void periodic() {
    // if (LineBreakSensorSubsystem.NoteStore.isNoted()) {
    //   setLEDPattern(LEDPattern.ORANGE);
    // }
  }
}
