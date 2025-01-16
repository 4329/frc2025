package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveByController;
import frc.robot.subsystems.LilihSubsystem;
import frc.robot.subsystems.LoggingSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.CommandLoginator;
import frc.robot.utilities.HoorayConfig;
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

  final SendableChooser<Command> m_chooser;

  // The driver's controllers
  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   *
   * @param drivetrain
   * @param lightSubsystem
   */
  public RobotContainer(Drivetrain drivetrain) {
    m_robotDrive = drivetrain;

    operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);
    driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

    m_robotDrive.setDefaultCommand(new DriveByController(drivetrain, driverController));

    lilihSubsystem = new LilihSubsystem();
    poseEstimationSubsystem = new PoseEstimationSubsystem(drivetrain, lilihSubsystem);

    loggingSubsystem = new LoggingSubsystem(poseEstimationSubsystem);

    new CommandLoginator();

    
    configureButtonBindings();
    configureAutoBuilder();

    m_chooser = new SendableChooser<>();
    configureAutoChooser(drivetrain);
  }

  // /** Creates and establishes camera streams for the shuffleboard ~Ben */
  // HttpCamera limelight;

  // private void initializeCamera() {

  //   // CameraServer.startAutomaticCapture();
  //   // // System.out.println(CameraServer.getVideo());
  //   // VideoSource[] enumerateSources = VideoSource.enumerateSources();
  //   // System.out.println(enumerateSources[0].getName());
  //   // if (enumerateSources.length > 0 &&
  //   // enumerateSources[0].getName().contains("USB")) {
  //   // Shuffleboard.getTab("RobotData").add("Camera",
  //   // enumerateSources[0]).withPosition(5, 0).withSize(3, 3)
  //   // .withWidget(BuiltInWidgets.kCameraStream);
  //   // }

  //   limelight = new HttpCamera("Limelight", HoorayConfig.gimmeConfig().getLimelighturl());
  //   System.out.println(HoorayConfig.gimmeConfig().getLimelighturl());
  //   CameraServer.startAutomaticCapture(limelight);
  //   // Shuffleboard.getTab("RobotData").add("Limelight Camera",
  //   // limelight).withPosition(2, 0).withSize(2, 2)
  //   // .withWidget(BuiltInWidgets.kCameraStream);
  // }

  private void configureAutoBuilder() {
    RobotConfig config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(
        poseEstimationSubsystem::getPathPlannerStuff,
        poseEstimationSubsystem::setInitialPose,
        m_robotDrive::getChassisSpeed,
        (speeds, feedForwards) -> m_robotDrive.setModuleStates(speeds),
        new PPHolonomicDriveController(
            Constants.AutoConstants.translationPID,
            Constants.AutoConstants.rotationPID),
        config,
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
    driverController.rightStick().onTrue(new UnInstantCommand(() -> m_robotDrive.resetOdometry(m_robotDrive.getPose())));
  }

  // spotless:on

  // jonathan was here today 2/3/2023
  /* Pulls autos and configures the chooser */
  // SwerveAutoBuilder swerveAutoBuilder;
  Map<Command, PathPlannerAuto> autoName = new HashMap<>();

  private void configureAutoChooser(Drivetrain drivetrain) {
    File pathPlannerDirectory = new File(Filesystem.getDeployDirectory(), "pathplanner");
    pathPlannerDirectory = new File(pathPlannerDirectory, "autos");

    for (File pathFile : pathPlannerDirectory.listFiles()) {

      if (pathFile.isFile() && pathFile.getName().endsWith(".auto")) {

        String name = pathFile.getName().replace(".auto", "");
        PathPlannerAuto pathCommand = new PathPlannerAuto(name);
        Command autoCommand =
            new SequentialCommandGroup(
                pathCommand,
                new InstantCommand(drivetrain::stop));
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
}
