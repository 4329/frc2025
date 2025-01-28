package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveByController;
import frc.robot.commands.algeePivotCommands.RunAlgeePivotCommand;
import frc.robot.commands.driveCommands.CenterByButtonRingCommand;
import frc.robot.commands.driveCommands.CenterOnTargetCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeeWheelSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LoggingSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmFactory;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.lilih.LilihSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.ButtonRingController;
import frc.robot.utilities.CommandLoginator;
import frc.robot.utilities.UnInstantCommand;
import java.io.File;
import java.util.HashMap;
import java.util.Map;

/* (including subsystems, commands, and button mappings) should be declared here
 */
public class RobotContainer {

  // The robot's subsystems
  private final Drivetrain m_robotDrive;
  private final PoseEstimationSubsystem poseEstimationSubsystem;
  private final LilihSubsystem lilihSubsystem;
  private final LoggingSubsystem loggingSubsystem;
  private final DifferentialArmSubsystem differentialArmSubsystem;
  private final AlgeePivotSubsystem algeePivotSubsystem;
  private final AlgeeWheelSubsystem algeeWheelSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;

  private final DriveByController driveByController;

  final SendableChooser<Command> m_chooser;

  // The driver's controllers
  private final CommandXboxController driverController;

  private final ButtonRingController buttonRingController;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   *
   * @param drivetrain
   * @param lightSubsystem
   */
  public RobotContainer(Drivetrain drivetrain) {
    m_robotDrive = drivetrain;

    driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    buttonRingController = new ButtonRingController(OIConstants.kOperatorControllerPort);

    driveByController = new DriveByController(drivetrain, driverController);
    m_robotDrive.setDefaultCommand(driveByController);

    lilihSubsystem = new LilihSubsystem(11, "limelight-lilih");
    poseEstimationSubsystem = new PoseEstimationSubsystem(drivetrain, lilihSubsystem);
    differentialArmSubsystem = DifferentialArmFactory.createDifferentialArmSubsystem();
    algeePivotSubsystem = new AlgeePivotSubsystem();
    algeeWheelSubsystem = new AlgeeWheelSubsystem();
    elevatorSubsystem = new ElevatorSubsystem();

    loggingSubsystem =
        new LoggingSubsystem(
            poseEstimationSubsystem, differentialArmSubsystem, buttonRingController);

    new CommandLoginator();

    configureButtonBindings();
    configureAutoBuilder();

    m_chooser = new SendableChooser<>();
    configureAutoChooser(drivetrain);
  }

  private void configureAutoBuilder() {
    try {
      Constants.AutoConstants.config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(
        poseEstimationSubsystem::getPose,
        poseEstimationSubsystem::setInitialPose,
        m_robotDrive::getChassisSpeed,
        (speeds, feedForwards) -> {
          m_robotDrive.setModuleStates(speeds);
        },
        Constants.AutoConstants.ppHolonomicDriveController,
        Constants.AutoConstants.config,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          throw new RuntimeException();
        },
        m_robotDrive);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  // spotless:off

  private void configureButtonBindings() {
    // driverController.start().onTrue(new UnInstantCommand(driveByController::changeFieldOrient));

    driverController.leftTrigger(0.01).whileTrue(new RepeatCommand(new UnInstantCommand(
      () -> elevatorSubsystem.runElevator(-driverController.getLeftTriggerAxis()))));
    driverController.rightTrigger(0.01).whileTrue(new RepeatCommand(new UnInstantCommand(
      () -> elevatorSubsystem.runElevator(driverController.getRightTriggerAxis()))));

    driverController.leftBumper().whileTrue(new RepeatCommand(new UnInstantCommand(
      () -> algeePivotSubsystem.run(-1))));
    driverController.rightBumper().whileTrue(new RepeatCommand(new UnInstantCommand(
      () -> algeePivotSubsystem.run(1))));

    driverController.a().whileTrue(new CenterByButtonRingCommand(poseEstimationSubsystem, m_robotDrive, buttonRingController));

    driverController.povUp().whileTrue(new RunAlgeePivotCommand(algeePivotSubsystem, 1));
    driverController.povDown().whileTrue(new RunAlgeePivotCommand(algeePivotSubsystem, -1));

    driverController.rightStick().onTrue(new InstantCommand(
      () -> m_robotDrive.resetOdometry(new Pose2d())));
  }

  // spotless:on

  // jonathan was here today 2/3/2023
  // benjamin e. was here today 1/18/2025
  /* Pulls autos and configures the chooser */
  // SwerveAutoBuilder swerveAutoBuilder;
  Map<Command, PathPlannerAuto> autoName = new HashMap<>();

  private void configureAutoChooser(Drivetrain drivetrain) {
    File pathPlannerDirectory = new File(Filesystem.getDeployDirectory(), "pathplanner");
    pathPlannerDirectory = new File(pathPlannerDirectory, "autos");
    if (pathPlannerDirectory.listFiles() == null) return;

    for (File pathFile : pathPlannerDirectory.listFiles()) {

      if (pathFile.isFile() && pathFile.getName().endsWith(".auto")) {

        String name = pathFile.getName().replace(".auto", "");
        PathPlannerAuto pathCommand = new PathPlannerAuto(name);
        Command autoCommand =
            new SequentialCommandGroup(pathCommand, new InstantCommand(drivetrain::stop));
        m_chooser.addOption(name, autoCommand);

        autoName.put(autoCommand, pathCommand);
      }
    }

    Shuffleboard.getTab("RobotData").add("SelectAuto", m_chooser).withSize(4, 2).withPosition(0, 0);
  }

  public void robotInit() {
    // new AutoZero(elevatorSubsystem, armAngleSubsystem).schedule();
    // limDriveSetCommand.schedule();
  }

  public void autonomousInit() {

    // limDriveSetCommand.schedule();
  }

  public void teleopInit() {
    // limDriveSetCommand.schedule();
    // autoZero.schedule();
  }

  public void autonomousPeriodic() {}

  public void teleopPeriodic() {}

  /**
   * @return Selected Auto
   */
  public Command getAuto() {
    return m_chooser.getSelected();
  }

  public void configureTestMode() {}

  public String getAutoName(Command command) {
    return autoName.containsKey(command) ? autoName.get(command).getName() : "Nothing?????/?///?";
  }

  public Map<Command, PathPlannerAuto> yes() {
    return autoName;
  }

  public void robotPeriodic() {
    if (lilihSubsystem.getTargetVisible(7)) {
      org.littletonrobotics.junction.Logger.recordOutput(
          "relPose", lilihSubsystem.getTargetPoseInRobotSpace(7));
    }
  }
}
