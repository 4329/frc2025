package frc.robot.subsystems.swerve.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Model.DrivetrainLogAutoLogged;
import org.littletonrobotics.junction.Logger;

public class DrivetrainSim extends DrivetrainImpl {
  private final Field2d field;
  private final DrivetrainLogAutoLogged log = new DrivetrainLogAutoLogged();

  public DrivetrainSim() {
    this.field = new Field2d();
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();
    field.setRobotPose(getPose());
    SmartDashboard.putData(field);

    log.pose = getPose();
    log.states = getModuleStates();

    Logger.processInputs("Drivetrain", log);
  }
}
