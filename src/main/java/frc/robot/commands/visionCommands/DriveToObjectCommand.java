package frc.robot.commands.visionCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lilih.LilihSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;

public class DriveToObjectCommand extends Command {

  Drivetrain drivetrain;
  LilihSubsystem lilihSubsystem;
  PIDController rotationPidController;
  PIDController drivePidController;

  public DriveToObjectCommand(Drivetrain drivetrain, LilihSubsystem lilihSubsystem) {
    this.drivetrain = drivetrain;
    this.lilihSubsystem = lilihSubsystem;

    rotationPidController = new PIDController(0.1, 0, 0);
    rotationPidController.setSetpoint(0);
    drivePidController = new PIDController(0.1, 0, 0);
    drivePidController.setSetpoint(0);

    addRequirements(drivetrain, lilihSubsystem);
  }

  @Override
  public void execute() {
    // if (!lilihSubsystem.cameraConnected() || lilihSubsystem.getLimelightResultsDetector() ==
    // null) {
    //   return;
    // }

    // double rotVar =
    //     rotationPidController.calculate(lilihSubsystem.getLimelightResultsDetector().tx);
    // drivetrain.drive(0, 0, rotVar, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
