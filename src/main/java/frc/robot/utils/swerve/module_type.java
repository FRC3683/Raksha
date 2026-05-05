package frc.robot.utils.swerve;

import edu.wpi.first.units.measure.LinearVelocity;
import static edu.wpi.first.units.Units.FeetPerSecond;

public enum module_type {
    mk4i_L1((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0), 150.0 / 7.0 , (25.0 / 19.0) * (15.0 / 45.0), FeetPerSecond.of(12.4)),
    mk4i_L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0), 150.0 / 7.0 , (27.0 / 17.0) * (15.0 / 45.0), FeetPerSecond.of(15.0)),
    mk4i_L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0), 150.0 / 7.0 , (28.0 / 16.0) * (15.0 / 45.0), FeetPerSecond.of(16.5)),
    mk5n_R1((54.0 / 12.0) * (25.0 / 32.0) * (30.0 / 15.0), 287.0 / 11.0, (32.0 / 25.0) * (15.0 / 30.0), FeetPerSecond.of(14.4)),
    mk5n_R2((54.0 / 14.0) * (25.0 / 32.0) * (30.0 / 15.0), 287.0 / 11.0, (32.0 / 25.0) * (15.0 / 30.0), FeetPerSecond.of(16.8)),
    mk5n_R3((54.0 / 16.0) * (25.0 / 32.0) * (30.0 / 15.0), 287.0 / 11.0, (32.0 / 25.0) * (15.0 / 30.0), FeetPerSecond.of(19.2)),
    ;

    public final double drive_ratio, steer_ratio, couple_ratio;
    module_type(double drive_ratio, double steer_ratio, double couple_ratio, LinearVelocity free_speed) {
        this.drive_ratio = drive_ratio;
        this.steer_ratio = steer_ratio;
        this.couple_ratio = couple_ratio;
    }
}
