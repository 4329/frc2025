package frc.robot.utilities;

import edu.wpi.first.wpilibj.shuffleboard.WidgetType;

public enum BuiltOutWidgets implements WidgetType {
  kRadiableGyro("RadiableGyro"),
  kSometimesText("Sometimes Text"),
  kLoadBar("Loading Bar");

  public String string;

  BuiltOutWidgets(String string) {
    this.string = string;
  }

  @Override
  public String getWidgetName() {
    return string;
  }
}
