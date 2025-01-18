package frc.robot.subsystems.swerve.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utilities.FieldRelativeAccel;
import frc.robot.utilities.FieldRelativeSpeed;

public interface Drivetrain extends Subsystem {

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative);

    void periodic();

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    void setModuleStates(SwerveModuleState[] desiredStates);

    void setModuleStates(ChassisSpeeds chassisSpeeds);

    void stop();

    /**
     * Updates odometry for the swerve drivetrain. This should be called once per loop to minimize
     * error.
     */
    void updateOdometry();

    /**
     * Function to retrieve latest robot gyro angle.
     *
     * @return Rotation2d object containing Gyro angle
     */
    Rotation2d getGyro();

    FieldRelativeSpeed getRelativeSpeed();

    FieldRelativeAccel getRelativeAccel();

    void setNesss(Pose2d set);

    Pose2d jgetNesss();

    /**
     * @return Pose2d object containing the X and Y position and the heading of the robot.
     */
    Pose2d getPose();

    /**
     * Resets the odometry and gyro to the specified pose.
     *
     * @param pose in which to set the odometry and gyro.
     */
    void resetOdometry(Pose2d pose);

    void setPose(Pose2d pose);

    /**
     * Converts the 4 swerve module states into a chassisSpeed by making use of the swerve drive
     * kinematics.
     *
     * @return ChassisSpeeds object containing robot X, Y, and Angular velocity
     */
    ChassisSpeeds getChassisSpeed();

    double getFrontLeftAngle();

    double getFrontRightAngle();

    double getBackLeftAngle();

    double getBackRightAngle();

    void brakeMode();

    void coastMode();

    void lock();

    void unlock();

    SwerveModulePosition[] getModulePositions();

    SwerveModuleState[] getModuleStates();

    double getRoll();

    double getOffsetRoll();

    double getYaw();
}
