package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

public class ElevatorSim extends ElevatorImpl {
	@AutoLogOutput
	private LoggedMechanism2d mechanism;

	public ElevatorSim() {
		super();

		mechanism = new LoggedMechanism2d(3, 3);
		mechanism.getRoot("root", 1.5, 0).append(new LoggedMechanismLigament2d("thing", 0.9, 90));
	}

}
