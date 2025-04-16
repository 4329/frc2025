package frc.robot.subsystems.swerve.module;

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
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.SparkFactory;

public class SwerveModuleSim extends SubsystemBase implements SwerveModule {
    private double m_driveCommand;
    private double m_turningCommand;

    private final DCMotorSim m_driveSim;
    private final DCMotorSim m_turningSim;

    private final PIDController m_drivePIDController;
    private final PIDController m_turningPIDController;

    // Creates a SimpleMotorFeedForward for the translation motor on the swerve
    // module
    // The static and feedforward gains should be passed into the class contructor
    // via the "tuningCals" array
    private SimpleMotorFeedforward driveFeedForward;

    public SwerveModuleSim(
            int driveMotorChannel,
            int turningMotorChannel,
            int turningEncoderChannel,
            double[] tuningVals) {

        DCMotor driveGearbox = DCMotor.getNEO(1);
        m_driveSim =
                new DCMotorSim(LinearSystemId.createDCMotorSystem(driveGearbox, 0.025, Constants.ModuleConstants.kTranslationGearRatio), driveGearbox);

        DCMotor turningGearbox = DCMotor.getNEO(1);
        m_turningSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(turningGearbox, .004, Constants.ModuleConstants.kTurningGearRatio), turningGearbox);

        // Creates the SimpleMotorFeedForward for the swerve module using the static and
        // feedforward gains from the tuningVals array
        driveFeedForward = new SimpleMotorFeedforward(tuningVals[0], tuningVals[1]);

        // Creates the drive PIDController using the proportional gain from the
        // tuningVals array
        m_drivePIDController = new PIDController(tuningVals[2], 0.0, 0.0);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous so the PID will command the shortest path.
        m_turningPIDController =
                new PIDController(
                        ModuleConstants.kTurnPID[0], ModuleConstants.kTurnPID[1], ModuleConstants.kTurnPID[2]);
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    private double getVelocity() {
        return m_driveSim.getAngularVelocityRPM() * Constants.ModuleConstants.kVelocityFactor;
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), new Rotation2d(getTurnEncoder()));
    }

    @Override
    public SwerveModuleState getStateNoOffset() {
        return getState();
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state =
                SwerveModuleState.optimize(desiredState, new Rotation2d(getTurnEncoder()));
        // Calculate the drive output from the drive PID controller.
        final double driveOutput =
                m_drivePIDController.calculate(getVelocity(), state.speedMetersPerSecond);
        // Calculates the desired feedForward motor % from the current desired velocity
        // and the static and feedforward gains
        final double driveFF = driveFeedForward.calculate(state.speedMetersPerSecond);
        // Set the drive motor to the sum of the feedforward calculation and PID
        // calculation
        final double finalDriveOutput = MathUtils.clamp(-1, 1, driveOutput + driveFF);

        m_driveCommand = finalDriveOutput;
        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput =
                MathUtils.clamp(
                        -1, 1, m_turningPIDController.calculate(getTurnEncoder(), state.angle.getRadians()));
        // Set the turning motor to this output value
        m_turningCommand = turnOutput;
    }

    @Override
    public void stop() {
        m_driveCommand = 0;
        m_turningCommand = 0;
    }

    @Override
    public double getTurnEncoder() {
        return m_turningSim.getAngularPositionRad();
    }

    @Override
    public void brakeModeModule() {
    }

    @Override
    public void coastModeModule() {
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveSim.getAngularPositionRotations() * Constants.ModuleConstants.kPositionFactor, new Rotation2d(getTurnEncoder()));
    }

    @Override
    public void simulationPeriodic() {
        m_driveSim.setInput(m_driveCommand * RoboRioSim.getVInVoltage());
        m_driveSim.update(.02);

        m_turningSim.setInput(m_turningCommand * RoboRioSim.getVInVoltage());
        m_turningSim.update(.02);

        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(
                        m_driveSim.getCurrentDrawAmps(), m_turningSim.getCurrentDrawAmps()));
    }
}
