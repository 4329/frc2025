package frc.robot.subsystems.differentialArm;

import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class DifferentialArmSim extends DifferentialArmImpl {

    SparkSim sparkSim1;
    SparkSim sparkSim2;

    SparkRelativeEncoderSim sparkRelativeEncoderSim1;
    SparkRelativeEncoderSim sparkRelativeEncoderSim2;

    private final DCMotorSim motorSim1;
    private final DCMotorSim motorSim2;

    public DifferentialArmSim() {
        DCMotor gearbox = DCMotor.getNeo550(1);
        sparkSim1 = new SparkSim(motor1, gearbox);
        sparkSim2 = new SparkSim(motor2, gearbox);

        sparkRelativeEncoderSim1 = sparkSim1.getRelativeEncoderSim();
        sparkRelativeEncoderSim2 = sparkSim2.getRelativeEncoderSim();

        motorSim1 = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.01, 1), gearbox);
        motorSim2 = new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.01, 1), gearbox);
    }

    @Override
    public double getPitch() {
        return (sparkRelativeEncoderSim1.getPosition() + sparkRelativeEncoderSim2.getPosition()) / 2;
    }

    @Override
    public double getRoll() {
        return sparkRelativeEncoderSim1.getPosition() - sparkRelativeEncoderSim2.getPosition();
    }

    @Override
    public void simulationPeriodic() {
        sparkSim1.setAppliedOutput(motor1.getAppliedOutput());
        motorSim1.setInput(sparkSim1.getAppliedOutput() * RoboRioSim.getVInVoltage());
        motorSim1.update(0.02);
        sparkSim1.iterate(motorSim1.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

        sparkSim2.setAppliedOutput(motor2.getAppliedOutput());
        motorSim2.setInput(sparkSim2.getAppliedOutput() * RoboRioSim.getVInVoltage());
        motorSim2.update(0.02);
        sparkSim2.iterate(motorSim2.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);
    }
}
