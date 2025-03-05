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
import frc.robot.Constants;
import frc.robot.model.AlgeePivotLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.subsystems.light.LEDState;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.SparkFactory;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class AlgeePivotSubsystem extends SubsystemBase implements LoggedSubsystem {
    private final double ALGEE_PIVOT_SPEED = 0.3;
    private GenericEntry speed = Shuffleboard.getTab("Asdf").add("aSpeed", 0).getEntry();
    private GenericEntry accel = Shuffleboard.getTab("Asdf").add("aAccel", 0).getEntry();

    private final double MIN = -10000;
    private final double MAX = 14000;

    public enum AlgeePivotAngle {
        ZERO(0),
        OUT(14),
        ;

        public double angle;

        AlgeePivotAngle(double angle) {
            this.angle = angle;
        }
    }

    private SparkMax motor;

    private ProfiledPIDController pidController;
    private TrapezoidProfile.Constraints profile = new TrapezoidProfile.Constraints(6, 2);

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
                                        .reverseSoftLimitEnabled(true))
                        .inverted(true);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        pidController = new ProfiledPIDController(.1, 0, 0, profile);
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
        return pidController.atSetpoint();
    }

    @Override
    public void periodic() {
        pidController.setConstraints(
                new TrapezoidProfile.Constraints(speed.getDouble(0), accel.getDouble(0)));

        motor.set(pidController.calculate(motor.getEncoder().getPosition()));
    }

    @Override
    public LoggableInputs log() {
        algeePivotLogAutoLogged.setpoint = pidController.getGoal().position;
        return algeePivotLogAutoLogged;
    }
}
