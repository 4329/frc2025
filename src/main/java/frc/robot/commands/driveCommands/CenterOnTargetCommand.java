package frc.robot.commands.driveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LilihSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.AprilTagUtil;

public class CenterOnTargetCommand extends Command {
  private final LilihSubsystem lilihSubsystem;
  private final Drivetrain drivetrain;
  private int targetId;

  private final PIDController rotationPID;

  public CenterOnTargetCommand(
      LilihSubsystem lilihSubsystem, Drivetrain m_drivetrain, int targetId) {
    this.lilihSubsystem = lilihSubsystem;
    this.drivetrain = m_drivetrain;
    this.targetId = targetId;

    rotationPID = new PIDController(0.033, 0, 0); // 0.75, 0, 0
    rotationPID.setTolerance(1);
    rotationPID.setSetpoint(0);

    addRequirements(lilihSubsystem, m_drivetrain);
  }

  @Override
  public void initialize() {
    rotationPID.reset();
    drivetrain.drive(0, 0, 0, true);
    targetId = AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker();
  }

  @Override
  public void execute() {
    double rotationCalc = 0;
    if (lilihSubsystem.CameraConnected() && lilihSubsystem.getTargetVisible(targetId)) {

      rotationCalc = rotationPID.calculate(lilihSubsystem.getTargetX(targetId));

      if (rotationCalc > Constants.DriveConstants.kMaxAngularSpeed) {
        rotationCalc = Constants.DriveConstants.kMaxAngularSpeed;
      } else if (rotationCalc < -Constants.DriveConstants.kMaxAngularSpeed) {
        rotationCalc = -Constants.DriveConstants.kMaxAngularSpeed;
      } else if (rotationPID.atSetpoint()) {
        rotationCalc = 0;
      }

      drivetrain.drive(0, 0, rotationCalc, true);
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
