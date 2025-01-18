package frc.robot.subsystems.swerve.drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DrivetrainSim extends DrivetrainImpl {
    private final Field2d field;

    public DrivetrainSim() {
        this.field = new Field2d();
    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
        field.setRobotPose(getPose());
        SmartDashboard.putData(field);
    }
}
