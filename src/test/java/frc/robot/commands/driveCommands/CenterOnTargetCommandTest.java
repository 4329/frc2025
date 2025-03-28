package frc.robot.commands.driveCommands;

import static org.mockito.Mockito.when;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.lilih.LilihSubsystem;
import frc.robot.subsystems.swerve.drivetrain.Drivetrain;
import frc.robot.utilities.CenterDistance;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
public class CenterOnTargetCommandTest {

    // @Mock Drivetrain drivetrain;
    // @Mock LilihSubsystem lilihSubsystem;
    // private final double clawOffset = 0.132;
    // CenterOnTargetCommand centerOnTargetCommand;

    // @BeforeEach
    // public void init() {
    //     when(drivetrain.getGyro()).thenReturn(new Rotation2d());
    //     when(drivetrain.getModulePositions()).thenReturn(new SwerveModulePosition[] {});
    //     centerOnTargetCommand =
    //             new CenterOnTargetCommand(
    //                     1, new PoseEstimationSubsystem(drivetrain, lilihSubsystem), drivetrain);
    // }

    // @Test
    // public void getNums() {
    //     AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    //     for (int i = 6; i <= 11; i++) {
    //         System.out.println(
    //                 i
    //                         + ": "
    //                         + centerOnTargetCommand.placeTarget(i, 0.1651 - clawOffset, CenterDistance.SCORING));
    //         System.out.println(
    //                 i
    //                         + ": "
    //                         + centerOnTargetCommand.placeTarget(i, -0.1651 - clawOffset, CenterDistance.SCORING));
    //     }
    // }
}
