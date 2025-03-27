package frc.robot.subsystems.elevator;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.model.ElevatorLogAutoLogged;

public class ElevatorNeoSim extends ElevatorNeo {

    private SparkMaxSim motorSim;
    private edu.wpi.first.wpilibj.simulation.ElevatorSim elevatorSim;

    private double volts;

    public ElevatorNeoSim() {

        DCMotor gearbox = DCMotor.getNEO(2);
        motorSim = new SparkMaxSim(motor1, gearbox);
        elevatorSim =
                new edu.wpi.first.wpilibj.simulation.ElevatorSim(
                        gearbox,
                        16,
                        Units.lbsToKilograms(25),
                        Units.inchesToMeters(1.788 / 2),
                        ElevatorSubsystem.MIN - ELEVATOR_START,
                        ElevatorSubsystem.MAX - ELEVATOR_START,
                        true,
                        0,
                        0.01,
                        0.0);
    }

    @Override
    public void simulationPeriodic() {
        elevatorSim.setInput(volts * RoboRioSim.getVInVoltage());
        elevatorSim.update(0.02);

        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    }

    @Override
    public void set(double speed) {
        volts = speed * 12;
    }

    @Override
    public void updateInputs(ElevatorLogAutoLogged inputs) {
        inputs.position = elevatorSim.getPositionMeters();
    }
}
