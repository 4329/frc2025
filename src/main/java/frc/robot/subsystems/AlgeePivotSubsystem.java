package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.model.AlgeePivotLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.SparkFactory;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class AlgeePivotSubsystem extends SubsystemBase implements LoggedSubsystem {

    public enum AlgeePivotAngle {
        ZERO(0),
        HIGH(Math.PI / 4),
        LOW(-Math.PI / 4);

        public double angle;

        AlgeePivotAngle(double angle) {
            this.angle = angle;
        }
    }

    private final double ALGEE_PIVOT_SPEED = 0.3;
    private final double MIN = 0;
    private final double MAX = 14;

    private SparkMax motor;
    private SparkBaseConfig config;

    private ProfiledPIDController pidController;
    private TrapezoidProfile.Constraints profile = new TrapezoidProfile.Constraints(6, 2);

    private final AlgeePivotLogAutoLogged algeePivotLogAutoLogged;

    private final GenericEntry maxVel = Shuffleboard.getTab("yep").add("vel", 0).getEntry();
    private final GenericEntry maxAccel = Shuffleboard.getTab("yep").add("accel", 0).getEntry();

    public AlgeePivotSubsystem() {
        motor = SparkFactory.createSparkMax(12);
        config =
                new SparkMaxConfig()
                        .apply(
                                new SoftLimitConfig()
                                        .forwardSoftLimit(MAX)
                                        .forwardSoftLimitEnabled(true)
                                        .reverseSoftLimit(MIN)
                                        .reverseSoftLimitEnabled(true));
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        pidController = new ProfiledPIDController(.1, 0, 0, profile);

        algeePivotLogAutoLogged = new AlgeePivotLogAutoLogged();
    }

    private void setSetpoint(double setpoint) {
        pidController.setGoal(MathUtils.clamp(MIN, MAX, setpoint));
    }

    public void run(double speed) {
        setSetpoint(pidController.getGoal().position + speed * ALGEE_PIVOT_SPEED);
    }

    public void setSetpoint(AlgeePivotAngle angle) {
        setSetpoint(angle.angle);
    }

    @Override
    public void periodic() {
        motor.set(pidController.calculate(motor.getEncoder().getPosition()));

        pidController.setConstraints(
                new TrapezoidProfile.Constraints(maxVel.getDouble(0), maxAccel.getDouble(0)));
    }

    @Override
    public LoggableInputs log() {
        algeePivotLogAutoLogged.setpoint = pidController.getGoal().position;
        return algeePivotLogAutoLogged;
    }
}
