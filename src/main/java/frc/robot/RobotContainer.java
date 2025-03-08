package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveByController;
import frc.robot.commands.algeePivotCommands.SetAlgeePivotCommand;
import frc.robot.commands.algeeWheelCommands.IntakeAlgeeCommand;
import frc.robot.commands.algeeWheelCommands.OuttakeAlgeeCommand;
import frc.robot.commands.commandGroups.ScoreWithArm;
import frc.robot.commands.commandGroups.StartCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import frc.robot.subsystems.AlgeeWheelSubsystem;
import frc.robot.subsystems.LoggingSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmFactory;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorFactory;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.light.LightSubsystem;
import frc.robot.subsystems.lilih.LilihSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.ButtonRingController;
import frc.robot.utilities.CommandLoginator;
import frc.robot.utilities.ToggleCommand;
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
    private final DifferentialArmSubsystem differentialArmSubsystem;
    private final AlgeePivotSubsystem algeePivotSubsystem;
    private final AlgeeWheelSubsystem algeeWheelSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final LightSubsystem lightSubsystem;
    // private final IntakeWheelSubsystem intakeWheelSubsystem;
    // private final IntakePivotSubsystem intakePivotSubsystem;

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
        Shuffleboard.getTab("RobotData")
                .add("Octagon", buttonRingController)
                .withPosition(4, 0)
                .withSize(3, 2);

        driveByController = new DriveByController(drivetrain, driverController);
        m_robotDrive.setDefaultCommand(driveByController);

        lilihSubsystem = new LilihSubsystem(11, "limelight-lilih");
        poseEstimationSubsystem = new PoseEstimationSubsystem(drivetrain, lilihSubsystem);
        differentialArmSubsystem = DifferentialArmFactory.createDifferentialArmSubsystem();
        algeePivotSubsystem = new AlgeePivotSubsystem();
        algeeWheelSubsystem = new AlgeeWheelSubsystem();
        // intakePivotSubsystem = new IntakePivotSubsystem();
        // intakeWheelSubsystem = new IntakeWheelSubsystem();
        elevatorSubsystem = ElevatorFactory.createElevatorSubsystem(differentialArmSubsystem::getPitch);
        lightSubsystem = new LightSubsystem();

        new LoggingSubsystem(
				drivetrain,
                poseEstimationSubsystem,
                differentialArmSubsystem,
                algeePivotSubsystem,
                elevatorSubsystem,
                buttonRingController,
                lightSubsystem);

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

	private final GenericEntry level = Shuffleboard.getTab("Asdf").add("sad", 0).getEntry();

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    // spotless:off

  private void configureButtonBindings() {
    driverController.start().onTrue(new UnInstantCommand("ToggleFieldOrient", driveByController::toggleFieldOrient));
    driverController.back().onTrue(new StartCommand(elevatorSubsystem, differentialArmSubsystem, algeePivotSubsystem));

    driverController.rightTrigger(0.01).whileTrue(new UnInstantCommand(
            "ElevatorUp",
            () -> elevatorSubsystem.runElevator(driverController.getRightTriggerAxis())).repeatedlyLog());
    driverController.leftTrigger(0.01).whileTrue(new UnInstantCommand(
            "ElevatorDown",
            () -> elevatorSubsystem.runElevator(-driverController.getLeftTriggerAxis())).repeatedlyLog());

	driverController.leftBumper().whileTrue(new SetAlgeePivotCommand(algeePivotSubsystem, AlgeePivotAngle.ZERO));
	driverController.rightBumper().whileTrue(new SetAlgeePivotCommand(algeePivotSubsystem, AlgeePivotAngle.OUT));

    driverController.a().whileTrue(new ScoreWithArm(algeePivotSubsystem, elevatorSubsystem, buttonRingController, differentialArmSubsystem, poseEstimationSubsystem, m_robotDrive));
	driverController.b().whileTrue(new ToggleCommand(new StartEndCommand(() -> elevatorSubsystem.setSetpoint(ElevatorSubsystem.ElevatorPosition.values()[(int)level.getDouble(0) * 2]), () -> elevatorSubsystem.setSetpoint(ElevatorSubsystem.ElevatorPosition.values()[((int)level.getDouble(0) + 1) * 2]))));
	driverController.x().onTrue(new IntakeAlgeeCommand(algeeWheelSubsystem, 1));
	driverController.y().whileTrue(new OuttakeAlgeeCommand(algeeWheelSubsystem));

    driverController.povUp().whileTrue(new RepeatCommand(new UnInstantCommand(
            "ArmPitchUp",
            () -> differentialArmSubsystem.runPitch(1))));
    driverController.povDown().whileTrue(new RepeatCommand(new UnInstantCommand(
            "ArmPitchDown",
            () -> differentialArmSubsystem.runPitch(-1))));

	driverController.povRight().onTrue(new UnInstantCommand(
				"Dif90",
				() -> differentialArmSubsystem.setPitchTarget(DifferentialArmSubsystem.DifferentialArmPitch.NINETY)
			));
	driverController.povLeft().onTrue(new UnInstantCommand(
				"Dif135",
				() -> differentialArmSubsystem.setPitchTarget(DifferentialArmSubsystem.DifferentialArmPitch.ONETHIRTYFIVE)
			));

    driverController.rightStick().onTrue(new UnInstantCommand(
            "ResetOdometry",
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
