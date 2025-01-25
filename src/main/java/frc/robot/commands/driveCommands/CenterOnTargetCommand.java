package frc.robot.commands.driveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.lilih.LilihSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import org.littletonrobotics.junction.Logger;

public class CenterOnTargetCommand extends Command {
  private final LilihSubsystem lilihSubsystem;
  private final Drivetrain drivetrain;
  private int targetId;

  private final PIDController rotationPID;
  private final PIDController xPID;
  private final PIDController yPID;

  public CenterOnTargetCommand(
      LilihSubsystem lilihSubsystem, Drivetrain m_drivetrain, int targetId) {
    this.lilihSubsystem = lilihSubsystem;
    this.drivetrain = m_drivetrain;
    this.targetId = targetId;

    rotationPID = new PIDController(0.033, 0, 0); // 0.75, 0, 0
    rotationPID.setTolerance(0.01);
    rotationPID.setSetpoint(0);

    xPID = new PIDController(1, 0, 0);
    xPID.setSetpoint(0);
    xPID.setTolerance(0);

    yPID = new PIDController(1, 0, 0);
    yPID.setSetpoint(1.1);
    yPID.setTolerance(0.1);

    addRequirements(lilihSubsystem, m_drivetrain);

    Shuffleboard.getTab("CenterTuning").add("X", xPID);
    Shuffleboard.getTab("CenterTuning").add("Y", yPID);
    Shuffleboard.getTab("CenterTuning").add("Rotation", rotationPID);
  }

  @Override
  public void initialize() {
    rotationPID.reset();
    drivetrain.stop();
  }

  @Override
  public void execute() {
    if (lilihSubsystem.cameraConnected() && lilihSubsystem.getTargetVisible(targetId)) {

      double rotationCalc =
          rotationPID.calculate(
              lilihSubsystem
                  .getTargetPoseInRobotSpace(targetId)
                  .getRotation()
                  .getMeasureY()
                  .in(Units.Radians));
      double xCalc = xPID.calculate(lilihSubsystem.getTargetPoseInRobotSpace(targetId).getX());
      double yCalc = -yPID.calculate(lilihSubsystem.getTargetPoseInRobotSpace(targetId).getZ());

      if (rotationCalc > Constants.DriveConstants.kMaxAngularSpeed) {
        rotationCalc = Constants.DriveConstants.kMaxAngularSpeed;
      } else if (rotationCalc < -Constants.DriveConstants.kMaxAngularSpeed) {
        rotationCalc = -Constants.DriveConstants.kMaxAngularSpeed;
      } else if (rotationPID.atSetpoint()) {
        rotationCalc = 0;
      }

      if (xPID.atSetpoint()) xCalc = 0;
      if (yPID.atSetpoint()) yCalc = 0;

      Logger.recordOutput("xCalc", xCalc);
      Logger.recordOutput("yCalc", yCalc);
      Logger.recordOutput("rotationCalc", rotationCalc);

      drivetrain.drive(yCalc, xCalc, rotationCalc, true);
    } else {
      drivetrain.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }
}
