package frc.robot.subsystems.differentialArm;

import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class DifferentialArmSim extends DifferentialArmImpl {

    SparkSim sparkSim1;
    SparkRelativeEncoderSim sparkRelativeEncoderSim1;

    private final DCMotorSim motorSim1;

    public DifferentialArmSim() {
        DCMotor gearbox = DCMotor.getNeo550(2);
        sparkSim1 = new SparkSim(motor1, gearbox);

        sparkRelativeEncoderSim1 = sparkSim1.getRelativeEncoderSim();

        motorSim1 = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.01, 1), gearbox);
    }

    @Override
    public double getPitch() {
        return sparkRelativeEncoderSim1.getPosition();
    }

    @Override
    public void simulationPeriodic() {
        sparkSim1.setAppliedOutput(motor1.getAppliedOutput());
        motorSim1.setInput(sparkSim1.getAppliedOutput() * RoboRioSim.getVInVoltage());
        motorSim1.update(0.02);
        sparkSim1.iterate(motorSim1.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);
    }
}
