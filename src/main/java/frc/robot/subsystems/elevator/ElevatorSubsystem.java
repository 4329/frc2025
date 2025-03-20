package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.model.ElevatorLogAutoLogged;
import frc.robot.utilities.HoorayConfig;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.shufflebored.ShuffledTrapezoidController;

import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {

    static final double MIN = -7.2;
    static final double MAX = 98.5;

    public enum ElevatorPosition {
        L2(57.4),
        L2Score(10),

        L3(125.46),
        L3Score(57.81),

        L4(196),
        L4AGAIN(L4.pos),

        ALGEE_LOW(30.26),
        ALGEE_HIGH(103.41),

        NET(97),

        ZERO(0),
        PORCESSOR(
                -23), // WIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIP

        DIFFERENTIAL_ARM_OUT(7.8),
        ALGEE_CLAW_OUT(-4.18),
        ;

        double pos;

        ElevatorPosition(double pos) {
            this.pos = pos;
        }
    }

    ElevatorLogAutoLogged inputs = new ElevatorLogAutoLogged();
    ElevatorIO io;

    private double ELEVATOR_SPEED = 2;

    ProfiledPIDController elevatorPID;

    public ElevatorSubsystem() {
        io =
                switch (Constants.robotMode) {
                    case REAL -> HoorayConfig.gimmeConfig().getIsElevatorNeo()
                            ? new ElevatorNeo()
                            : new ElevatorFalcon();
                    case SIM -> new ElevatorNeoSim();
                    default -> new ElevatorIO() {};
                };

        elevatorPID = new ShuffledTrapezoidController(0.09, 0, 0, new TrapezoidProfile.Constraints(160, 240));
		Shuffleboard.getTab("Asdf").add("asdf", elevatorPID);
        elevatorPID.setTolerance(1);
    }

    public void setSetpoint(ElevatorPosition setpoint) {
		setSetpoint(setpoint.pos);
    }

    private void setSetpoint(double setpoint) {
        elevatorPID.setGoal(MathUtils.clamp(MIN, MAX, setpoint));
    }

    public void runElevator(double dir) {
        setSetpoint(elevatorPID.getGoal().position + ELEVATOR_SPEED * dir);
    }

    public boolean atSetpoint() {
        return elevatorPID.atGoal();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        inputs.setpoint = elevatorPID.getGoal().position;
        inputs.atSetpoint = atSetpoint();
        Logger.processInputs("ElevatorSubsystem", inputs);

        io.set(MathUtils.clamp(-1, 1, elevatorPID.calculate(inputs.position)));
    }
}
