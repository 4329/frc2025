package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utilities.CommandLoginator;

/* (including subsystems, commands, and button mappings) should be declared here
 */
public class RobotContainer {

  // The robot's subsystems
  private final Drivetrain m_robotDrive;

  final SendableChooser<Command> m_chooser;

  // The driver's controllers
  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;
  private final CommandXboxController pitController;

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
    pitController = new CommandXboxController(OIConstants.kPitControllerPort);
    // commands for auto
    // Command Instantiations
    new CommandLoginator();

    m_chooser = new SendableChooser<>();
    // initializeCamera();
    configureButtonBindings();
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
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  // spotless:off

  private void configureButtonBindings() {
  }

  // spotless:on

  // jonathan was here today 2/3/2023
  /* Pulls autos and configures the chooser */
  // SwerveAutoBuilder swerveAutoBuilder;
  Map<Command, PathPlannerAuto> autoName = new HashMap<>();

  private void configureAutoChooser(Drivetrain drivetrain) {
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

  public void configureTestMode() {

  }

  public String getAutoName(Command command) {
    return autoName.containsKey(command) ? autoName.get(command).getName() : "Nothing?????/?///?";
  }

  public Map<Command, PathPlannerAuto> yes() {
    return autoName;
  }
}
