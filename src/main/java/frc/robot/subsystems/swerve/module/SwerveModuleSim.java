package frc.robot.subsystems.swerve.module;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.utilities.SparkFactory;

public class SwerveModuleSim implements SwerveModule {

    private final SparkMaxSim m_driveMotor;
    private final SparkMaxSim m_turningMotor;

    private final SparkMax m_driveMotorSpark;
    private final SparkMax m_turningMotorSpark;

    private final SparkRelativeEncoderSim m_driveEncoder;
    private final SparkRelativeEncoderSim m_turningEncoder;

    private final SparkBaseConfig m_driveConfig;
    private final SparkBaseConfig m_turningConfig;

    private final PIDController m_drivePIDController;

    // Creates a SimpleMotorFeedForward for the translation motor on the swerve
    // module
    // The static and feedforward gains should be passed into the class contructor
    // via the "tuningCals" array
    private SimpleMotorFeedforward driveFeedForward;

    // Creates a PIDController for the control of the anglular position of the
    // swerve module
    private final PIDController m_turningPIDController = new PIDController(
            ModuleConstants.kTurnPID[0], ModuleConstants.kTurnPID[1], ModuleConstants.kTurnPID[2]);

    public SwerveModuleSim(
            int driveMotorChannel,
            int turningMotorChannel,
            int turningEncoderChannel,
            double[] tuningVals) {

        // with the input driveMotorChannel
        m_driveMotorSpark = SparkFactory.createSparkMax(
                driveMotorChannel, false); // Define the drive motor as the SparkMAX

        m_driveConfig = new SparkMaxConfig()
                .smartCurrentLimit(
                        ModuleConstants.kDriveCurrentLimit) // Set current limit for the drive motor
                .voltageCompensation(
                        DriveConstants.kVoltCompensation); // Enable voltage compensation so

        m_driveConfig.encoder
                .positionConversionFactor(ModuleConstants.kPositionFactor)
                // Set velocity conversion factor so
                // that encoder and PID control is in
                // terms of velocity in m/s
                .velocityConversionFactor(ModuleConstants.kVelocityFactor);

        m_driveMotorSpark.configure(
                m_driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_driveMotor = new SparkMaxSim(m_driveMotorSpark, DCMotor.getNEO(1));

        m_driveEncoder = m_driveMotor.getRelativeEncoderSim();

        m_turningMotorSpark = SparkFactory.createSparkMax(turningMotorChannel, false); // Define the drive motor
                                                                                              // as the
        // SparkMAX with the input
        // driveMotorChannel

        m_turningConfig = new SparkMaxConfig()
                .smartCurrentLimit(ModuleConstants.kTurnCurrentLimit)
                .voltageCompensation(DriveConstants.kVoltCompensation);

        m_turningMotorSpark.configure(
                m_turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_turningMotor = new SparkMaxSim(m_turningMotorSpark, DCMotor.getNEO(1));

        // Creates the analog potentiometer for the tracking of the swerve module
        // position converted to the range of 0-2*PI in radians offset by the tuned
        // module offset
        m_turningEncoder = m_turningMotor.getRelativeEncoderSim();

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous so the PID will command the shortest path.
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // Creates the SimpleMotorFeedForward for the swerve module using the static and
        // feedforward gains from the tuningVals array
        driveFeedForward = new SimpleMotorFeedforward(tuningVals[0], tuningVals[1]);

        // Creates the drive PIDController using the proportional gain from the
        // tuningVals array
        m_drivePIDController = new PIDController(tuningVals[2], 0.0, 0.0);
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveMotor.getVelocity(), new Rotation2d(getTurnEncoder()));
    }

    @Override
    public SwerveModuleState getStateNoOffset() {
        return getState();
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getTurnEncoder()));
        // Calculate the drive output from the drive PID controller.
        final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity(),
                state.speedMetersPerSecond);
        // Calculates the desired feedForward motor % from the current desired velocity
        // and the static and feedforward gains
        final double driveFF = driveFeedForward.calculate(state.speedMetersPerSecond);
        // Set the drive motor to the sum of the feedforward calculation and PID
        // calculation
        final double finalDriveOutput = driveOutput + driveFF;
        m_driveMotor.setAppliedOutput(finalDriveOutput);
        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = m_turningPIDController.calculate(getTurnEncoder(), state.angle.getRadians());
        // Set the turning motor to this output value
        m_turningMotor.setAppliedOutput(-turnOutput);
    }

    @Override
    public void stop() {
        m_driveMotor.setAppliedOutput(0);
        m_turningMotor.setAppliedOutput(0);
    }

    @Override
    public double getTurnEncoder() {
        return m_turningEncoder.getPosition() % Math.PI;
    }

    @Override
    public void brakeModeModule() {
        m_driveConfig.idleMode(IdleMode.kBrake);
        m_turningConfig.idleMode(IdleMode.kBrake);
        m_driveMotorSpark.configure(
                m_driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_turningMotorSpark.configure(
                m_turningConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void coastModeModule() {
        m_driveConfig.idleMode(IdleMode.kCoast);
        m_turningConfig.idleMode(IdleMode.kCoast);
        m_driveMotorSpark.configure(
                m_driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_turningMotorSpark.configure(
                m_turningConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveMotor.getPosition(), new Rotation2d(getTurnEncoder()));
    }

}
