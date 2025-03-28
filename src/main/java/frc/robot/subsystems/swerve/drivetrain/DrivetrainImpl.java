// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.drivetrain;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.DriveConstants;
import frc.robot.model.DrivetrainLogAutoLogged;
import frc.robot.subsystems.swerve.module.SwerveModule;
import frc.robot.subsystems.swerve.module.SwerveModuleFactory;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Implements a swerve DrivetrainImpl Subsystem for the Robot */
public class DrivetrainImpl extends SubsystemBase implements Drivetrain {

    public boolean isLocked;
    private double offset;

    private final SwerveModule m_frontLeft;
    private final SwerveModule m_frontRight;
    private final SwerveModule m_backLeft;
    private final SwerveModule m_backRight;
    private final DrivetrainLogAutoLogged log = new DrivetrainLogAutoLogged();

    // Create the PIDController for the Keep Angle PID
    private final PIDController m_keepAnglePID =
            new PIDController(
                    DriveConstants.kKeepAnglePID[0],
                    DriveConstants.kKeepAnglePID[1],
                    DriveConstants.kKeepAnglePID[2]);

    protected double keepAngle = 0.0; // Double to store the current target keepAngle in radians
    private double timeSinceRot = 0.0; // Double to store the time since last rotation command
    private double lastRotTime = 0.0; // Double to store the time of the last rotation command
    private double timeSinceDrive = 0.0; // Double to store the time since last translation command
    private double lastDriveTime = 0.0; // Double to store the time of the last translation command

    private final Timer keepAngleTimer =
            new Timer(); // Creates timer used in the perform keep angle function
    // Creates a swerveModule object for the front left swerve module feeding in
    // parameters from the constants file
    // Creates an ahrs gyro (NavX) on the MXP port of the RoboRIO
    private static AHRS ahrs = new AHRS(NavXComType.kMXP_SPI);

    // Creates Odometry object to store the pose of the robot
    protected final SwerveDriveOdometry m_odometry;

    private SlewRateLimiter slewX = new SlewRateLimiter(5); // 6.5
    private SlewRateLimiter slewY = new SlewRateLimiter(5);
    private SlewRateLimiter slewRot = new SlewRateLimiter(7); // 10

    /** Constructs a DrivetrainImpl and resets the Gyro and Keep Angle parameters */
    public DrivetrainImpl() {
        keepAngleTimer.reset();
        keepAngleTimer.start();
        m_keepAnglePID.enableContinuousInput(-Math.PI, Math.PI);
        ahrs.reset();

        offset = Math.PI;

        m_frontLeft =
                SwerveModuleFactory.makeSwerve(
                        DriveConstants.kFrontLeftDriveMotorPort,
                        DriveConstants.kFrontLeftTurningMotorPort,
                        DriveConstants.kFrontLeftTurningEncoderPort,
                        DriveConstants.kFrontLeftOffset,
                        DriveConstants.kFrontLeftTuningVals);

        m_frontRight =
                SwerveModuleFactory.makeSwerve(
                        DriveConstants.kFrontRightDriveMotorPort,
                        DriveConstants.kFrontRightTurningMotorPort,
                        DriveConstants.kFrontRightTurningEncoderPort,
                        DriveConstants.kFrontRightOffset,
                        DriveConstants.kFrontRightTuningVals);

        m_backLeft =
                SwerveModuleFactory.makeSwerve(
                        DriveConstants.kBackLeftDriveMotorPort,
                        DriveConstants.kBackLeftTurningMotorPort,
                        DriveConstants.kBackLeftTurningEncoderPort,
                        DriveConstants.kBackLeftOffset,
                        DriveConstants.kBackLeftTuningVals);

        m_backRight =
                SwerveModuleFactory.makeSwerve(
                        DriveConstants.kBackRightDriveMotorPort,
                        DriveConstants.kBackRightTurningMotorPort,
                        DriveConstants.kBackRightTurningEncoderPort,
                        DriveConstants.kBackRightOffset,
                        DriveConstants.kBackRightTuningVals);

        m_odometry =
                new SwerveDriveOdometry(
                        DriveConstants.kDriveKinematics, ahrs.getRotation2d(), getModulePositions());
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    @Override
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        rot =
                performKeepAngle(
                        xSpeed, ySpeed,
                        rot); // Calls the keep angle function to update the keep angle or rotate
        // depending on driver input
        xSpeed = slewX.calculate(xSpeed);
        ySpeed = slewY.calculate(ySpeed);
        rot = slewRot.calculate(rot);

        // creates an array of the desired swerve module states based on driver command
        // and if the commands are field relative or not
        SwerveModuleState[] swerveModuleStates =
                DriveConstants.kDriveKinematics.toSwerveModuleStates(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyro())
                                : new ChassisSpeeds(xSpeed, ySpeed, rot));

        // normalize wheel speeds so all individual states are scaled to achievable
        // velocities
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

        if (!isLocked) {

            setModuleStates(swerveModuleStates);
        }
    }

    @Override
    public void periodic() {
        // Update swerve drive odometry periodically so robot pose can be tracked
        updateOdometry();
    }

    @Override
    public LoggableInputs log() {
        log.states = getModuleStates();
        log.rot = getGyro();
        log.offset = offset;

        return log;
    }

    @Override
    public void simulationPeriodic() {
        ahrs.setAngleAdjustment(
                ahrs.getAngle()
                        + (DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates())
                                                .omegaRadiansPerSecond
                                        * 180
                                        / Math.PI)
                                * 0.02);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    @Override
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_backLeft.setDesiredState(desiredStates[2]);
        m_backRight.setDesiredState(desiredStates[3]);
    }

    @Override
    public void setModuleStates(ChassisSpeeds chassisSpeeds) {
        setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    @Override
    public void stop() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_backLeft.stop();
        m_backRight.stop();
    }

    /**
     * Updates odometry for the swerve drivetrain. This should be called once per loop to minimize
     * error.
     */
    @Override
    public void updateOdometry() {

        m_odometry.update(getGyro(), getModulePositions());
    }

    @Override
    public void setInitialRotation(double initial) {
        ahrs.reset();
        ahrs.setAngleAdjustment(initial);
    }

    /**
     * Function to retrieve latest robot gyro angle.
     *
     * @return Rotation2d object containing Gyro angle
     */
    @Override
    public Rotation2d getGyro() {
        return ahrs.getRotation2d().plus(new Rotation2d(offset));
    }

    @Override
    public Rotation2d getRawGyro() {
        return ahrs.getRotation2d();
    }

    /**
     * @return Pose2d object containing the X and Y position and the heading of the robot.
     */
    @Override
    public Pose2d getPose() {
        Pose2d initialPose = m_odometry.getPoseMeters();
        return new Pose2d(
                initialPose.getX(),
                initialPose.getY(),
                new Rotation2d(initialPose.getRotation().getRadians()));
    }

    /**
     * Resets the odometry and gyro to the specified pose.
     *
     * @param pose in which to set the odometry and gyro.
     */
    @Override
    public void resetOdometry(Pose2d pose) {
        offset = -getRawGyro().plus(pose.getRotation()).getRadians();
        keepAngle = getGyro().getRadians();
        m_odometry.resetPosition(ahrs.getRotation2d(), getModulePositions(), pose);
    }

    @Override
    public void setPose(Pose2d pose) {
        m_odometry.resetPosition(ahrs.getRotation2d().times(-1.0), getModulePositions(), pose);
        keepAngle = getGyro().getRadians();
    }

    /**
     * Converts the 4 swerve module states into a chassisSpeed by making use of the swerve drive
     * kinematics.
     *
     * @return ChassisSpeeds object containing robot X, Y, and Angular velocity
     */
    @Override
    public ChassisSpeeds getChassisSpeed() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_backLeft.getState(),
                m_backRight.getState());
    }

    /**
     * Keep angle function is performed to combat drivetrain drift without the need of constant
     * "micro-adjustments" from the driver. A PIDController is used to attempt to maintain the robot
     * heading to the keepAngle value. This value is updated when the robot is rotated manually by the
     * driver input
     *
     * @return rotation command in radians/s
     * @param xSpeed is the input drive X speed command
     * @param ySpeed is the input drive Y speed command
     * @param rot is the input drive rotation speed command
     */
    private double performKeepAngle(double xSpeed, double ySpeed, double rot) {
        double output =
                rot; // Output shouold be set to the input rot command unless the Keep Angle PID is
        // called
        if (Math.abs(rot)
                >= DriveConstants
                        .kMinRotationCommand) { // If the driver commands the robot to rotate set the
            // last rotate time to the current time
            lastRotTime = keepAngleTimer.get();
        }
        if (Math.abs(xSpeed) >= DriveConstants.kMinTranslationCommand
                || Math.abs(ySpeed)
                        >= DriveConstants
                                .kMinTranslationCommand) { // if driver commands robot to translate set the
            // last drive time to the current time
            lastDriveTime = keepAngleTimer.get();
        }

        timeSinceRot =
                keepAngleTimer.get()
                        - lastRotTime; // update variable to the current time - the last rotate time
        timeSinceDrive =
                keepAngleTimer.get()
                        - lastDriveTime; // update variable to the current time - the last drive time
        if (timeSinceRot
                < 0.5) { // Update keepAngle up until 0.5s after rotate command stops to allow rotation
            // move to finish
            keepAngle = getGyro().getRadians();
        } else if (Math.abs(rot) < DriveConstants.kMinRotationCommand
                && timeSinceDrive < 0.75) { // Run Keep angle pid
            // until 0.75s after drive
            // command stops to combat
            // decel drift
            // TODO understand this and make not negative (- = stupid)
            output =
                    m_keepAnglePID.calculate(
                            getGyro().getRadians(), keepAngle); // Set output command to the result of the
            // Keep Angle PID
        }
        return output;
    }

    @Override
    public double getFrontLeftAngle() {
        return m_frontLeft.getTurnEncoder();
    }

    @Override
    public double getFrontRightAngle() {
        return m_frontRight.getTurnEncoder();
    }

    @Override
    public double getBackLeftAngle() {
        return m_backLeft.getTurnEncoder();
    }

    @Override
    public double getBackRightAngle() {
        return m_backRight.getTurnEncoder();
    }

    @Override
    public void brakeMode() {
        m_frontLeft.brakeModeModule();
        m_frontRight.brakeModeModule();
        m_backLeft.brakeModeModule();
        m_backRight.brakeModeModule();
    }

    @Override
    public void coastMode() {
        m_frontLeft.coastModeModule();
        m_frontRight.coastModeModule();
        m_backLeft.coastModeModule();
        m_backRight.coastModeModule();
    }

    @Override
    public void lock() {

        SwerveModuleState[] steve = {
            new SwerveModuleState(0, Rotation2d.fromDegrees(225)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(225))
        };

        setModuleStates(steve);
        isLocked = true;
    }

    @Override
    public void unlock() {

        isLocked = false;
    }

    @Override
    public SwerveModulePosition[] getModulePositions() {

        return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        };
    }

    @Override
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(), m_backRight.getState()
        };
    }

    @Override
    public void resetKeepAngle() {
        keepAngle = getGyro().getRadians();
    }
}
