package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ElevatorSim extends ElevatorImpl {

	private SparkMaxSim motorSim;
	private edu.wpi.first.wpilibj.simulation.ElevatorSim elevatorSim;


	public ElevatorSim() {
		super();

		DCMotor gearbox = DCMotor.getNEO(1);
		motorSim = new SparkMaxSim(motor1, gearbox);
		elevatorSim = new edu.wpi.first.wpilibj.simulation.ElevatorSim(
				gearbox,
				1,
				100,
				Units.inchesToMeters(1.788 / 2),
				0,
				100,
				true,
				0,
				0.01,
				0.0);

	}

	@Override
	public void simulationPeriodic() {
	}
}
