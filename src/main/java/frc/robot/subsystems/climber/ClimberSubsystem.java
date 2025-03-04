package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.model.ClimberLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.SparkFactory;
import frc.robot.utilities.shufflebored.ShuffledPIDController;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimberSubsystem extends SubsystemBase implements LoggedSubsystem {
    private final double CLIMBY_SPEEDY = 0.5; // WIP

    private final double ZERO = 0;
    private final double MIN = -99999; // WIP
    private final double MAX = 99999; // WIP

    private double setpoint = 0;

    public enum ClimberPosition {
        DEPLOYED(10), // WIP
        ZERO(0), // WIP
        CLIMBED(-10); // WIP
        private double pos;
        ClimberPosition(double climberPosition) {
            this.pos = climberPosition;
        }

        public double getPos() {
            return pos;
        }
    }

    private ClimberLogAutoLogged climberLogAutoLogged = new ClimberLogAutoLogged();

    SparkMax motor;
    SparkBaseConfig config;
    RelativeEncoder encoder;

    private PIDController pidController;

    public ClimberSubsystem() {
        motor = SparkFactory.createSparkMax(Constants.SparkIDs.climber);
        encoder = motor.getEncoder();
        config =
                new SparkMaxConfig()
                        .apply(
                                new SoftLimitConfig()
                                        .forwardSoftLimit(MAX)
                                        .forwardSoftLimitEnabled(true)
                                        .reverseSoftLimit(MIN)
                                        .reverseSoftLimitEnabled(true));
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        pidController = new ShuffledPIDController(1, 0, 0); // WIP

        climberLogAutoLogged = new ClimberLogAutoLogged();
    }

    private void setSetpoint(double settyPointy) {
        this.setpoint = MathUtils.clamp(MIN, MAX, settyPointy);
        pidController.setSetpoint(this.setpoint);
    }

    public void setSetpoint(ClimberPosition climby) {
        setSetpoint(climby.getPos());
    }

    public void run(double speed) {
        setSetpoint(setpoint + (speed * CLIMBY_SPEEDY));
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    @Override
    public void periodic() {
        motor.set(pidController.calculate(motor.getEncoder().getPosition()));
    }

    @Override
    public LoggableInputs log() {
        climberLogAutoLogged.position = encoder.getPosition();
        return climberLogAutoLogged;
    }
}
