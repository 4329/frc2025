package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.model.ElevatorLogAutoLogged;
import frc.robot.utilities.HoorayConfig;
import frc.robot.utilities.MathUtils;

public class ElevatorSubsystem extends SubsystemBase {

    static final double MIN = -25;
    static final double MAX = 264;

    public enum ElevatorPosition {
        L2(57.4),
        L2Score(10),

        L3(125.46),
        L3Score(57.81),

        L4(196),
        L4AGAIN(L4.pos),

        ALGEE_LOW(30.26),
        ALGEE_HIGH(103.41),

        NET(263),

        ZERO(0),
        PORCESSOR(
                -23), // WIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIPWIP

        DIFFERENTIAL_ARM_OUT(19.7),
        ALGEE_CLAW_OUT(-14.4),
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
		io = switch (Constants.robotMode) {
			case REAL -> HoorayConfig.gimmeConfig().isElevatorNeo() ? new ElevatorNeo() : new ElevatorFalcon();
            case SIM -> new ElevatorNeoSim();
            default -> new ElevatorIO() {};
        };

		elevatorPID = new ProfiledPIDController(0.09, 0, 0, new TrapezoidProfile.Constraints(160, 240));
		elevatorPID.setTolerance(1);
	}


	public void setSetpoint(ElevatorPosition setpoint) {
		elevatorPID.setGoal(setpoint.pos);
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
