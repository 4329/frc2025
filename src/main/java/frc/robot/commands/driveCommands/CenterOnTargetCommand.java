package frc.robot.commands.driveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.lilih.LilihSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.ShuffledPIDController;
import org.littletonrobotics.junction.Logger;

public class CenterOnTargetCommand extends Command {
  private final LilihSubsystem lilihSubsystem;
  private final Drivetrain drivetrain;
  private int targetId;

  private final ShuffledPIDController rotationPID;
  private final ShuffledPIDController xPID;
  private final ShuffledPIDController yPID;

  private Pose2d initOdometry;
  private Pose2d newOdometry;

  public CenterOnTargetCommand(
      LilihSubsystem lilihSubsystem, Drivetrain m_drivetrain, int targetId) {
    this.lilihSubsystem = lilihSubsystem;
    this.drivetrain = m_drivetrain;
    this.targetId = targetId;

    rotationPID = new ShuffledPIDController(1, 0, 0); // 0.75, 0, 0
    rotationPID.setTolerance(0.01);
    rotationPID.setSetpoint(0);

    xPID = new ShuffledPIDController(1, 0, 0);
    xPID.setSetpoint(0);
    xPID.setTolerance(0);

    yPID = new ShuffledPIDController(1, 0, 0);
    yPID.setSetpoint(1.1);
    yPID.setTolerance(0.1);

    addRequirements(lilihSubsystem, m_drivetrain);

    Shuffleboard.getTab("CenterTuning").add("X", xPID);
    Shuffleboard.getTab("CenterTuning").add("Y", yPID);
    Shuffleboard.getTab("CenterTuning").add("Rotation", rotationPID);
  }

  @Override
  public void initialize() {
    if (!lilihSubsystem.cameraConnected() || !lilihSubsystem.getTargetVisible(targetId)) {
      return;
    }

    rotationPID.reset();
    drivetrain.stop();
    initOdometry = drivetrain.getPose();
    newOdometry = lilihSubsystem.getTargetPoseInFieldSpace(targetId);
    drivetrain.resetOdometry(
        new Pose2d(
            newOdometry.getTranslation(),
            new Rotation2d(
                lilihSubsystem
                    .getTargetPoseInRobotSpace(targetId)
                    .getRotation()
                    .getMeasureY()
                    .in(Units.Radians))));
  }

  @Override
  public void execute() {
    if (lilihSubsystem.cameraConnected() && lilihSubsystem.getTargetVisible(targetId)) {

      Logger.recordOutput("taginfieldspace", lilihSubsystem.getTargetPoseInFieldSpace(targetId));

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
    return !lilihSubsystem.cameraConnected() || !lilihSubsystem.getTargetVisible(targetId);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.resetOdometry(initOdometry);
    drivetrain.stop();
  }
}
