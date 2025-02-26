package frc.robot.subsystems.elevator;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ElevatorSim extends ElevatorImpl {

    private SparkMaxSim motorSim;
    private edu.wpi.first.wpilibj.simulation.ElevatorSim elevatorSim;

    public ElevatorSim() {
        super();

        DCMotor gearbox = DCMotor.getNEO(1);
        motorSim = new SparkMaxSim(motor1, gearbox);
        elevatorSim =
                new edu.wpi.first.wpilibj.simulation.ElevatorSim(
                        gearbox,
                        1,
                        100,
                        Units.inchesToMeters(1.788 / 2),
                        MIN,
                        MAX,
                        true,
                        ELEVATOR_START,
                        0.01,
                        0.0);
    }

    @Override
    public void simulationPeriodic() {
        elevatorSim.setInput(motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        elevatorSim.update(0.02);

        motorSim.iterate(elevatorSim.getVelocityMetersPerSecond(), RoboRioSim.getVInVoltage(), 0.02);

        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    }
}
