// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.module;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.*;
import frc.robot.subsystems.swerve.module.encoderNonsense.EncoderNonsense;
import frc.robot.subsystems.swerve.module.encoderNonsense.ReduxEncoder;
import frc.robot.subsystems.swerve.module.encoderNonsense.ThriftyEncoder;
import frc.robot.utilities.HoorayConfig;
import frc.robot.utilities.SparkFactory;
import org.littletonrobotics.junction.Logger;

/** Implements a swerve module for the Robot */
public class SwerveModuleImpl implements SwerveModule {

    // Our swerve modules use NEOs for both translation and rotation motors
    private final SparkMax m_driveMotor;
    private final SparkMax m_turningMotor;

    // Create a RelativeEncoder object for the translation position and velocity
    private final RelativeEncoder m_driveEncoder;

    // Create a Potentiometer to store the output of the absolute encoder that
    // tracks the angular position of the swerve module
    // private final AnalogPotentiometer m_turningEncoder;
    private final EncoderNonsense m_turningEncoder;

    private final SparkBaseConfig m_driveConfig;
    private final SparkBaseConfig m_turningConfig;

    // Creates a variable to store the moduleID for various tuning and debugging
    // (Currently not being used)
    // This value should be passed into the class contructor as part of the
    // "tuningVals" array
    private final double moduleID;

    // Creates a PIDController for the translation motor on the swerve module
    // The PID values should be passed into the class constructor via the
    // "tuningVals" array where they will be set
    private final PIDController m_drivePIDController;

    // Creates a SimpleMotorFeedForward for the translation motor on the swerve
    // module
    // The static and feedforward gains should be passed into the class contructor
    // via the "tuningCals" array
    private SimpleMotorFeedforward driveFeedForward;

    // Creates a PIDController for the control of the anglular position of the
    // swerve module
    private final PIDController m_turningPIDController =
            new PIDController(
                    ModuleConstants.kTurnPID[0], ModuleConstants.kTurnPID[1], ModuleConstants.kTurnPID[2]);

    private double angularOffset;

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, and turning encoder.
     *
     * @param driveMotorChannel CAN ID for the drive motor.
     * @param turningMotorChannel CAN ID for the turning motor.
     * @param turningEncoderChannel analog input for turning absolute encoder
     * @param angularOffset module specific offset for the absolute encoder
     * @param tuningVals double array containing tuning values for translation in the following format
     *     {StaticGain, FeedForward, Prop Gain, ModuleID}
     */
    public SwerveModuleImpl(
            int driveMotorChannel,
            int turningMotorChannel,
            int turningEncoderChannel,
            double angularOffset,
            double[] tuningVals) {

        // with the input driveMotorChannel
        m_driveMotor =
                SparkFactory.createSparkMax(
                        driveMotorChannel, false); // Define the drive motor as the SparkMAX

        m_driveConfig =
                new SparkMaxConfig()
                        .smartCurrentLimit(
                                ModuleConstants.kDriveCurrentLimit) // Set current limit for the drive motor
                        .voltageCompensation(
                                DriveConstants.kVoltCompensation); // Enable voltage compensation so

        m_driveConfig
                .encoder
                .positionConversionFactor(ModuleConstants.kPositionFactor)
                // Set velocity conversion factor so
                // that encoder and PID control is in
                // terms of velocity in m/s
                .velocityConversionFactor(ModuleConstants.kVelocityFactor);

        m_driveMotor.configure(
                m_driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // feedforward and gains scale with bus
        // voltage
        // Motor direction is not inverted
        m_driveEncoder = m_driveMotor.getEncoder(); // Obtain the driveEncoder from the drive SparkMAX

        m_turningMotor =
                SparkFactory.createSparkMax(
                        turningMotorChannel, false, false); // Define the drive motor as the
        // SparkMAX with the input
        // driveMotorChannel

        m_turningConfig =
                new SparkMaxConfig()
                        .smartCurrentLimit(ModuleConstants.kTurnCurrentLimit)
                        .voltageCompensation(DriveConstants.kVoltCompensation);

        // Creates the analog potentiometer for the tracking of the swerve module
        // position converted to the range of 0-2*PI in radians offset by the tuned
        // module offset
        // m_turningEncoder = new AnalogPotentiometer(turningEncoderChannel, 2.0 * Math.PI,
        // angularOffset);
        m_turningEncoder =
                switch (HoorayConfig.gimmeConfig().getEncoderType()) {
                    case REDUX -> new ReduxEncoder(m_turningConfig, m_turningMotor.getAbsoluteEncoder());
                    case THRIFTY -> new ThriftyEncoder(turningEncoderChannel);
                };

        m_turningMotor.configure(
                m_turningConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        this.angularOffset = angularOffset;

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous so the PID will command the shortest path.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        m_turningPIDController.setTolerance(0);

        // Creates the SimpleMotorFeedForward for the swerve module using the static and
        // feedforward gains from the tuningVals array
        driveFeedForward = new SimpleMotorFeedforward(tuningVals[0], tuningVals[1]);

        // Creates the drive PIDController using the proportional gain from the
        // tuningVals array
        m_drivePIDController = new PIDController(tuningVals[2], 0.0, 0.0);

        // Sets the moduleID to the value stored in the tuningVals array
        moduleID = tuningVals[3];
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(getTurnEncoder()));
    }

    @Override
    public SwerveModuleState getStateNoOffset() {
        return new SwerveModuleState(
                m_driveEncoder.getVelocity(), new Rotation2d(getTurnEncoder() - angularOffset));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        desiredState.optimize(new Rotation2d(getTurnEncoder()));
        // Calculate the drive output from the drive PID controller.
        final double driveOutput =
                m_drivePIDController.calculate(
                        m_driveEncoder.getVelocity(), desiredState.speedMetersPerSecond);
        // Calculates the desired feedForward motor % from the current desired velocity
        // and the static and feedforward gains
        final double driveFF = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
        // Set the drive motor to the sum of the feedforward calculation and PID
        // calculation
        final double finalDriveOutput = driveOutput + driveFF;
        m_driveMotor.set(finalDriveOutput);
        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput =
                m_turningPIDController.calculate(getTurnEncoder(), desiredState.angle.getRadians());
        Logger.recordOutput("" + moduleID, m_turningPIDController.getError());
        // Set the turning motor to this output value
        m_turningMotor.set(-turnOutput);
    }

    @Override
    public void stop() {
        m_driveMotor.set(0.0);
        m_turningMotor.set(0.0);
    }

    /**
     * Obtains the negative of the turning absolute encoder value as this encoder reads opposite of
     * the module rotation on 2910 MK2 swerve.
     *
     * @return the modified absolute encoder value.
     */
    @Override
    public double getTurnEncoder() {
        return m_turningEncoder.get();
    }

    @Override
    public void brakeModeModule() {
        m_driveConfig.idleMode(IdleMode.kBrake);
        m_turningConfig.idleMode(IdleMode.kBrake);
        m_driveMotor.configure(
                m_driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_turningMotor.configure(
                m_turningConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void coastModeModule() {
        m_driveConfig.idleMode(IdleMode.kCoast);
        m_turningConfig.idleMode(IdleMode.kCoast);
        m_driveMotor.configure(
                m_driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_turningMotor.configure(
                m_turningConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public SwerveModulePosition getPosition() {

        return new SwerveModulePosition(m_driveEncoder.getPosition(), new Rotation2d(getTurnEncoder()));
    }
}
