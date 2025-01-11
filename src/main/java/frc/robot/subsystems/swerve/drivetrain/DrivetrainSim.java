package frc.robot.subsystems.swerve.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Model.DrivetrainLog;
import frc.robot.Model.DrivetrainLogAutoLogged;
import frc.robot.subsystems.swerve.module.SwerveModule;
import frc.robot.subsystems.swerve.module.SwerveModuleFactory;
import frc.robot.utilities.FieldRelativeAccel;
import frc.robot.utilities.FieldRelativeSpeed;

public class DrivetrainSim extends DrivetrainImpl {
    private final Field2d field;
    private final DrivetrainLogAutoLogged log = new DrivetrainLogAutoLogged();

    public DrivetrainSim() {
        this.field = new Field2d();
    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
        field.setRobotPose(getPose());
        SmartDashboard.putData(field);
        
        log.pose = getPose();
        log.states = getModuleStates();

        Logger.processInputs("Drivetrain", log);
    }
}
