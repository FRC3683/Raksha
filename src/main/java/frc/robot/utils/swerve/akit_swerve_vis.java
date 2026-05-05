package frc.robot.utils.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.constants;
import frc.robot.utils.swerve.swerve_kin2.module_output;
import frc.robot.utils.swerve.swerve_kin2.module_speed;

public class akit_swerve_vis {
    private final SwerveModuleState[] desired = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };
    private final SwerveModuleState[] actual = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    private StructArrayPublisher<SwerveModuleState> desired_publisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("akit/swerve/desired_states", SwerveModuleState.struct).publish();

        private StructArrayPublisher<SwerveModuleState> actual_publisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("akit/swerve/actual_states", SwerveModuleState.struct).publish();

        private StructPublisher<Rotation2d> heading_publisher = NetworkTableInstance.getDefault()
        .getStructTopic("akit/swerve/heading", Rotation2d.struct).publish();

    public void update(Rotation2d heading, module_output[] desired, module_speed[] current) {
        for(int i = 0; i < 4; ++i) {
            this.desired[i].speedMetersPerSecond = desired[i].drive_output * constants.swerve.max_module_speed_mps;
            this.desired[i].angle = Rotation2d.fromRadians(desired[i].theta_rad);
            this.actual[i].speedMetersPerSecond = current[i].drive_speed_mps;
            this.actual[i].angle = Rotation2d.fromRadians(current[i].theta_rad);
        }
        desired_publisher.set(this.desired);
        actual_publisher.set(this.actual);
        heading_publisher.set(heading);
    }
}
