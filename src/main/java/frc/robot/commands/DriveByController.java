package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.MathUtils;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/** Implements a DriveByController command which extends the Command class */
public class DriveByController extends Command {
    private final Drivetrain drivetrain;
    private final CommandXboxController m_controller;
    private boolean fieldOrient = false;
    private GenericEntry fieldOrientStatus =
            Shuffleboard.getTab("RobotData")
                    .add("Field Orient On", true)
                    .withProperties(Map.of("Color when true", "#FFFFFF", "Color when false", "#000000"))
                    .withSize(4, 2)
                    .withPosition(0, 2)
                    .getEntry();

    private final boolean logStuff;

    /**
     * Contructs a DriveByController object which applys the driver inputs from the controller to the
     * swerve drivetrain
     *
     * @param drive is the swerve drivetrain object which should be created in the RobotContainer
     *     class
     * @param controller is the user input controller object for controlling the drivetrain
     */
    public DriveByController(Drivetrain drive, CommandXboxController controller) {
        drivetrain = drive;
        m_controller = controller;

        // Because this will be used as a default command, add the subsystem which will use this as the
        // default
        addRequirements(drivetrain);
        logStuff = true;
    }

    public DriveByController(Drivetrain drive, CommandXboxController controller, boolean logStuff) {
        drivetrain = drive;
        m_controller = controller;

        // Because this will be used as a default command, add the subsystem which will use this as the
        // default
        addRequirements(drivetrain);
        this.logStuff = logStuff;
    }

    /** the execute function is overloaded with the function to drive the swerve drivetrain; */
    @Override
    public void execute() {
        // :3
        drivetrain.drive(
                -inputTransform(m_controller.getLeftY()) * DriveConstants.kMaxSpeedMetersPerSecond,
                -inputTransform(m_controller.getLeftX()) * DriveConstants.kMaxSpeedMetersPerSecond,
                -inputTransform(m_controller.getRightX()) * DriveConstants.kMaxAngularSpeed,
                fieldOrient);

        // m_controller.setRumble(
        //         RumbleType.kBothRumble,
        //         MathUtils.clamp(
        //                         0,
        //                         1,
        //                         Math.sin(DriverStation.getMatchTime()) * m_robotDrive.getVelocity() /
        // Constants.DriveConstants.kMaxSpeedMetersPerSecond));

        if (logStuff) Logger.recordOutput("Field Oriented", fieldOrient);
    }

    public void end(boolean interrpted) {
        m_controller.setRumble(RumbleType.kBothRumble, 0);
    }

    /**
     * when this fucntion of the command is called the current fieldOrient boolean is flipped. This is
     * fed into the drive command for the swerve drivetrain so the driver can decide to drive in a
     * robot oreinted when they please (not recommended in most instances)
     */
    public void turnOnFieldOrient() {
        fieldOrient = true;
        if (logStuff) fieldOrientStatus.setBoolean(true);
    }

    public void turnOnRobotOrient() {
        fieldOrient = false;
        if (logStuff) fieldOrientStatus.setBoolean(false);
    }

    public void toggleFieldOrient() {
        if (fieldOrient == true) {
            turnOnRobotOrient();
        } else {
            turnOnFieldOrient();
        }
    }

    /**
     * This function takes the user input from the controller analog sticks, applys a deadband and
     * then quadratically transforms the input so that it is easier for the user to drive, this is
     * especially important on high torque motors such as the NEOs or Falcons as it makes it more
     * intuitive and easier to make small corrections
     *
     * @param input is the input value from the controller axis, should be a value between -1.0 and
     *     1.0
     * @return the transformed input value
     */
    private double inputTransform(double input) {
        return MathUtils.singedPow(MathUtils.applyDeadband(input), 2);
    }
}
