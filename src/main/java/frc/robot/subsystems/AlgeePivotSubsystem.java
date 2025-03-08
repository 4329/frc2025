package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.model.AlgeePivotLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.subsystems.light.LEDState;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.SparkFactory;
import frc.robot.utilities.shufflebored.ShuffledTrapezoidController;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class AlgeePivotSubsystem extends SubsystemBase implements LoggedSubsystem {
    private final double ALGEE_PIVOT_SPEED = 0.3;

    private final double MIN = 0;
    private final double MAX = 27.9;

    public enum AlgeePivotAngle {
        ZERO(0),
        OUT(19),
        ;

        public double angle;

        AlgeePivotAngle(double angle) {
            this.angle = angle;
        }
    }

    private SparkMax motor;

    private ProfiledPIDController pidController;
    private TrapezoidProfile.Constraints profile = new TrapezoidProfile.Constraints(85, 145);

    private final AlgeePivotLogAutoLogged algeePivotLogAutoLogged;

    public AlgeePivotSubsystem() {
        motor = SparkFactory.createSparkMax(Constants.SparkIDs.algeePivot);
        SparkBaseConfig config =
                new SparkMaxConfig()
                        .apply(
                                new SoftLimitConfig()
                                        .forwardSoftLimit(MAX)
                                        .forwardSoftLimitEnabled(true)
                                        .reverseSoftLimit(MIN)
                                        .reverseSoftLimitEnabled(true));
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        pidController = new ShuffledTrapezoidController(.1, 0, 0, profile);
        pidController.setTolerance(0.5);
        Shuffleboard.getTab("Asdf").add("apivot", pidController);

        algeePivotLogAutoLogged = new AlgeePivotLogAutoLogged();
    }

    private void setSetpoint(double setpoint) {
        if (!LEDState.algeeWheelHolding) pidController.setGoal(MathUtils.clamp(MIN, MAX, setpoint));
    }

    public void run(double speed) {
        setSetpoint(pidController.getGoal().position + speed * ALGEE_PIVOT_SPEED);
    }

    public void setSetpoint(AlgeePivotAngle angle) {
        setSetpoint(angle.angle);
    }

    public boolean atSetpoint() {
        return pidController.atGoal();
    }

    @Override
    public void periodic() {
        motor.set(pidController.calculate(motor.getEncoder().getPosition()));
    }

    @Override
    public LoggableInputs log() {
        algeePivotLogAutoLogged.setpoint = pidController.getGoal().position;
        algeePivotLogAutoLogged.actual = motor.getEncoder().getPosition();
        algeePivotLogAutoLogged.atSetpoint = atSetpoint();

        return algeePivotLogAutoLogged;
    }
}
