package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveByController;
import frc.robot.commands.algeePivotCommands.RunAlgeePivotCommand;
import frc.robot.commands.algeePivotCommands.SetAlgeePivotCommand;
import frc.robot.commands.algeeWheelCommands.IntakeAlgeeCommand;
import frc.robot.commands.algeeWheelCommands.OuttakeAlgeeCommand;
import frc.robot.commands.autoCommands.AutoActuallyScoreCoralCommand;
import frc.robot.commands.autoCommands.AutoPositionCoralCommand;
import frc.robot.commands.autoCommands.AutoScoreCoralButCool;
import frc.robot.commands.commandGroups.AlgeeIntake;
import frc.robot.commands.commandGroups.HPStationCommand;
import frc.robot.commands.commandGroups.HappyResetCommand;
import frc.robot.commands.commandGroups.ScoreCoralCommand;
import frc.robot.commands.commandGroups.ScoreWithArm;
import frc.robot.commands.commandGroups.StartCommand;
import frc.robot.commands.differentialArmCommands.SetArmPitchCommand;
import frc.robot.commands.elevatorCommands.CoolEvator;
import frc.robot.commands.elevatorCommands.SetElevatorCommand;
import frc.robot.subsystems.AlgeePivotSubsystem;
import frc.robot.subsystems.AlgeePivotSubsystem.AlgeePivotAngle;
import frc.robot.subsystems.AlgeeWheelSubsystem;
import frc.robot.subsystems.LoggingSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmFactory;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem;
import frc.robot.subsystems.differentialArm.DifferentialArmSubsystem.DifferentialArmPitch;
import frc.robot.subsystems.elevator.ElevatorFactory;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.light.LightSubsystem;
import frc.robot.subsystems.lilih.LilihSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.ButtonRingController;
import frc.robot.utilities.CommandLoginator;
import frc.robot.utilities.ToggleCommand;
import frc.robot.utilities.UnInstantCommand;
import frc.robot.utilities.loggedComands.LoggedSequentialCommandGroup;
import java.io.File;
import java.util.HashMap;
import java.util.Map;
import frc.robot.commands.DoAFunctionalCommand;

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
    private final CommandXboxController manualController;
    private final CommandXboxController functionalController;

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
        manualController = new CommandXboxController(OIConstants.kManualControllerPort);
        functionalController = new CommandXboxController(OIConstants.kFunctionalControllerPort);
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
        elevatorSubsystem = ElevatorFactory.createElevatorSubsystem();
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
        configureNamedCommands();
        configureButtonBindings();
        configureAutoBuilder();

        m_chooser = new SendableChooser<>();
        configureAutoChooser(drivetrain);
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand("startCommand", new StartCommand(elevatorSubsystem, differentialArmSubsystem, algeePivotSubsystem));
        NamedCommands.registerCommand(
                "elevatorL2",
                new AutoPositionCoralCommand(
                        elevatorSubsystem, differentialArmSubsystem, ElevatorPosition.L2));
        NamedCommands.registerCommand(
                "elevatorScoreL2",
                new AutoActuallyScoreCoralCommand(
                        elevatorSubsystem, differentialArmSubsystem, ElevatorPosition.L2));

        NamedCommands.registerCommand(
                "elevatorL3",
                new AutoPositionCoralCommand(
                        elevatorSubsystem, differentialArmSubsystem, ElevatorPosition.L3));
        NamedCommands.registerCommand(
                "elevatorScoreL3",
                new AutoActuallyScoreCoralCommand(
                        elevatorSubsystem, differentialArmSubsystem, ElevatorPosition.L3));

        NamedCommands.registerCommand(
                "elevatorL4",
                new AutoPositionCoralCommand(
                        elevatorSubsystem, differentialArmSubsystem, ElevatorPosition.L4));
        NamedCommands.registerCommand(
                "elevatorScoreL4",
                new AutoActuallyScoreCoralCommand(
                        elevatorSubsystem, differentialArmSubsystem, ElevatorPosition.L4));

        NamedCommands.registerCommand(
                "intakeCoral",
                new HPStationCommand(differentialArmSubsystem, elevatorSubsystem, algeePivotSubsystem));
        NamedCommands.registerCommand("grabCoral", new SetElevatorCommand(elevatorSubsystem,ElevatorPosition.ZERO).andThen(new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.DIFFERENTIAL_ARM_OUT)));

        for (int i = 0; i < 6; i++) {
            addCool(i, ElevatorPosition.L2, ElevatorPosition.L2Score);
            addCool(i, ElevatorPosition.L3, ElevatorPosition.L3Score);
            addCool(i, ElevatorPosition.L4, ElevatorPosition.L4);
        }

        NamedCommands.registerCommand(
                "lowerArm", new SetArmPitchCommand(differentialArmSubsystem, DifferentialArmPitch.NINETY));
    }

    private void addCool(int num, ElevatorPosition position, ElevatorPosition scorePosition) {
        final String letters = "CBAFED";
        NamedCommands.registerCommand(
                "Side" + letters.charAt(num) + "Right" + position,
                new AutoScoreCoralButCool(
                        algeePivotSubsystem,
                        elevatorSubsystem,
                        position,
                        scorePosition,
                        differentialArmSubsystem,
                        poseEstimationSubsystem,
                        m_robotDrive,
                        num,
                        true));
        NamedCommands.registerCommand(
                "Side" + letters.charAt(num) + "Left" + position,
                new AutoScoreCoralButCool(
                        algeePivotSubsystem,
                        elevatorSubsystem,
                        position,
                        scorePosition,
                        differentialArmSubsystem,
                        poseEstimationSubsystem,
                        m_robotDrive,
                        num,
                        false));
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

		driverController.start().onTrue(new UnInstantCommand(
					"ToggleFieldOrient",
					driveByController::toggleFieldOrient
					));

        driverController.back().onTrue(new UnInstantCommand(
            "ResetRotation",
            poseEstimationSubsystem::resetRotOffset));

		driverController.rightTrigger(0.01).whileTrue(new UnInstantCommand(
					"ElevatorUp",
					() -> elevatorSubsystem.runElevator(driverController.getRightTriggerAxis())).repeatedlyLog());
		driverController.leftTrigger(0.01).whileTrue(new UnInstantCommand(
					"ElevatorDown",
					() -> elevatorSubsystem.runElevator(-driverController.getLeftTriggerAxis())).repeatedlyLog());

		driverController.rightBumper().whileTrue(new ScoreWithArm(algeePivotSubsystem, elevatorSubsystem, buttonRingController, differentialArmSubsystem, poseEstimationSubsystem, m_robotDrive));
		driverController.leftBumper().whileTrue(new ScoreCoralCommand(elevatorSubsystem, differentialArmSubsystem, buttonRingController));

		driverController.a().onTrue(new HPStationCommand(differentialArmSubsystem, elevatorSubsystem, algeePivotSubsystem));
		driverController.b().onTrue(new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.ZERO).andThenLog(new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.DIFFERENTIAL_ARM_OUT)));
		driverController.x().whileTrue(new AlgeeIntake(m_robotDrive, elevatorSubsystem, algeeWheelSubsystem, algeePivotSubsystem, poseEstimationSubsystem, buttonRingController));
		driverController.y().whileTrue(new OuttakeAlgeeCommand(algeeWheelSubsystem));

		driverController.povUp().onTrue(new SetElevatorCommand(elevatorSubsystem, ElevatorPosition.NET));
		driverController.povDown().onTrue(new HappyResetCommand(differentialArmSubsystem, elevatorSubsystem, algeePivotSubsystem));
		driverController.povRight().onTrue(new StartCommand(elevatorSubsystem, differentialArmSubsystem, algeePivotSubsystem));

		driverController.rightStick().onTrue(new UnInstantCommand(
					"ResetOdometry",
					() -> m_robotDrive.resetOdometry(new Pose2d())));

		manualController.start().onTrue(new SetAlgeePivotCommand(algeePivotSubsystem, AlgeePivotAngle.ZERO));
		manualController.back().onTrue(new SetAlgeePivotCommand(algeePivotSubsystem, AlgeePivotAngle.OUT));

		manualController.rightTrigger(0.01).whileTrue(new UnInstantCommand(
					"ElevatorUp",
					() -> elevatorSubsystem.runElevator(manualController.getRightTriggerAxis())).repeatedlyLog());
		manualController.leftTrigger(0.01).whileTrue(new UnInstantCommand(
					"ElevatorDown",
					() -> elevatorSubsystem.runElevator(-manualController.getLeftTriggerAxis())).repeatedlyLog());

		manualController.leftBumper().whileTrue(new RunAlgeePivotCommand(algeePivotSubsystem, 1));
		manualController.rightBumper().whileTrue(new RunAlgeePivotCommand(algeePivotSubsystem, -1));

		manualController.a().onTrue(new HPStationCommand(differentialArmSubsystem, elevatorSubsystem, algeePivotSubsystem));
		CoolEvator eleCool = new CoolEvator(elevatorSubsystem);
		manualController.b().whileTrue(new ToggleCommand(eleCool).untilLog(eleCool::isFinished));
		manualController.x().whileTrue(new IntakeAlgeeCommand(algeeWheelSubsystem));
		manualController.y().whileTrue(new OuttakeAlgeeCommand(algeeWheelSubsystem));

		manualController.povUp().whileTrue(new RepeatCommand(new UnInstantCommand(
						"ArmPitchUp",
						() -> differentialArmSubsystem.runPitch(1))));
		manualController.povDown().whileTrue(new RepeatCommand(new UnInstantCommand(
						"ArmPitchDown",
						() -> differentialArmSubsystem.runPitch(-1))));

		manualController.povRight().onTrue(new UnInstantCommand(
					"Dif90",
					() -> differentialArmSubsystem.setPitchTarget(DifferentialArmSubsystem.DifferentialArmPitch.NINETY)));
		manualController.povLeft().onTrue(new UnInstantCommand(
					"Dif0",
					() -> differentialArmSubsystem.setPitchTarget(DifferentialArmSubsystem.DifferentialArmPitch.STORAGE)));

		manualController.rightStick().onTrue(new UnInstantCommand(
					"Dif135",
					() -> differentialArmSubsystem.setPitchTarget(DifferentialArmSubsystem.DifferentialArmPitch.ONETHIRTYFIVE)));


        functionalController.a().onFalse(new DoAFunctionalCommand(m_robotDrive, functionalController.getHID(), elevatorSubsystem, differentialArmSubsystem, algeePivotSubsystem, algeeWheelSubsystem));
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
                        new SequentialCommandGroup(
                                pathCommand,
                                new InstantCommand(drivetrain::stop));
                m_chooser.addOption(name, autoCommand);

                autoName.put(autoCommand, pathCommand);
            }
        }

        SysIdRoutine routine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(null, null, null, null),
                        new SysIdRoutine.Mechanism(
                                differentialArmSubsystem::voltageDrive,
                                differentialArmSubsystem::logMotors,
                                differentialArmSubsystem));

        m_chooser.addOption(
                "diffid",
                new LoggedSequentialCommandGroup(
                        "diffid",
                        routine.quasistatic(Direction.kForward),
                        routine.quasistatic(Direction.kForward),
                        routine.dynamic(Direction.kForward),
                        routine.dynamic(Direction.kForward)));

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
