// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utilities.HoorayConfig;
import frc.robot.utilities.SwerveAlignment;
import java.io.File;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private SwerveAlignment m_swerveAlignment;
  private Drivetrain drivetrain;

  private Field2d field = new Field2d();

  Timer timer = new Timer();

  public Robot() {}

  private File findThumbDir() {
    File f = new File("/media");
    for (File kid : f.listFiles()) {
      File logs = new File(kid, "logs");
      if (logs.exists() && logs.canWrite()) {
        Logger.recordMetadata("Logging On:", "Flash Drive");
        return logs;
      } else if (logs.mkdir()) {
        Logger.recordMetadata("Logging On:", "Flash Drive");
        return logs;
      }
    }

    File homeDir = new File("/home/lvuser/logs");
    if (homeDir.exists() || homeDir.mkdir()) {
      Logger.recordMetadata("Logging On:", "Robot");
      return homeDir;
    } else {
      Logger.recordMetadata("Logging On:", "Nothing");
      return null;
    }
  }

  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
    if (isReal()) {
      File logFolder = findThumbDir();
      if (logFolder != null) {
        Logger.addDataReceiver(
            new WPILOGWriter(logFolder.getAbsolutePath())); // Log to a USB stick ("/U/logs")
      }
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      // new PowerDistribution(1, ModuleType.kRev); // Enables power distribution
      // logging
      Constants.robotMode = Mode.REAL;

    } else if (isSimulation()) {
      Logger.addDataReceiver(new NT4Publisher());
      Constants.robotMode = Mode.SIM;

    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath =
          LogFileUtil
              .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(
          new WPILOGWriter(
              LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
      Constants.robotMode = Mode.REPLAY;
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in
    // the "Understanding Data Flow" page
    // DataLogManager.start();
    // Logger.registerURCL(URCL.startExternal());
    Logger.start();

    HoorayConfig.gimmeConfig();
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.

    drivetrain = new Drivetrain();
    drivetrain.resetOdometry(new Pose2d());

    m_robotContainer = new RobotContainer(drivetrain);

    drivetrain.resetOdometry(new Pose2d());
    m_robotContainer.robotInit();
  }

  @Override
  public void robotPeriodic() {
    Logger.recordOutput("noPose", new Pose2d());
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    timer.start();
    // beforeMatchCommand.schedule();
    drivetrain.brakeMode();
  }

  String lastName = "";

  @Override
  public void disabledPeriodic() {
    // String name = m_robotContainer.getAutoName(m_robotContainer.getAuto());

    // if (name != lastName && !name.equals("Nothing?????/?///?")) {
    //   Trajectory accumulator = new Trajectory();
    //   List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(name);
    //   for (int i = 0; i < paths.size(); i++) {
    //     accumulator =
    //         accumulator.concatenate(
    //                 paths.get(i).getWaypoints().get(0));
    //   }
    //   field.getObject("traj").setTrajectory(accumulator);
    //   SmartDashboard.putData(field);
    // }
    // lastName = name;
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAuto();
    Logger.recordOutput("Auto", m_robotContainer.getAutoName(m_autonomousCommand));

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    m_robotContainer.autonomousInit();
  }

  @Override
  public void autonomousPeriodic() {

    m_robotContainer.autonomousPeriodic();
  }

  @Override
  public void autonomousExit() {

    Command resetForTeliOp =
        new InstantCommand(
            () -> drivetrain.resetOdometry(new Pose2d(new Translation2d(), new Rotation2d(0.0))));
    resetForTeliOp.schedule();
  }

  @Override
  public void teleopInit() {

    // be added.
    drivetrain.brakeMode();
    m_robotContainer.teleopInit();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

    m_robotContainer.teleopPeriodic();
  }

  @Override
  public void testInit() {

    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    drivetrain.coastMode();
    m_robotContainer.configureTestMode();

    if (m_swerveAlignment == null) {
      // This prevents 2 sets of widgets from appearing when disabling & enabling the
      // robot, causing a crash
      m_swerveAlignment = new SwerveAlignment(drivetrain);
      m_swerveAlignment.initSwerveAlignmentWidgets();
    }
  }

  @Override
  public void testPeriodic() {

    m_swerveAlignment.updateSwerveAlignment();
  }
}
