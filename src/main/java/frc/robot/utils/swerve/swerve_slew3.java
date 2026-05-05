// modified WPILIB SlewRateLimiter class to play nice with chassis speeds

package frc.robot.utils.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.utils.math_utils;

/**
 * TODO replace ChassisSpeeds with chassis_output and use this in swerve_lowlevel as a jerk limit
 **/
public class swerve_slew3 {

    private final double strafe_accel_limit; // max change per calculate call
    private final double omega_accel_limit; // max change per calculate call

    private final ChassisSpeeds cached = new ChassisSpeeds(0, 0, 0);

    public swerve_slew3(double strafe_accel_limit_s, double omega_accel_limit_s, double control_dts) {
        strafe_accel_limit = strafe_accel_limit_s * control_dts;
        omega_accel_limit = omega_accel_limit_s * control_dts;
    }

    public void calculate(ChassisSpeeds input, ChassisSpeeds prev, double throttle) {
        if(math_utils.hypot_squred(prev, input) < math_utils.hypot_squred(cached, input)) {
            cached.vxMetersPerSecond = prev.vxMetersPerSecond;
            cached.vyMetersPerSecond = prev.vyMetersPerSecond;
        }
        math_utils.move_toward(cached, input, strafe_accel_limit * throttle);
        input.vxMetersPerSecond = cached.vxMetersPerSecond;
        input.vyMetersPerSecond = cached.vyMetersPerSecond;

        if(Math.abs(prev.omegaRadiansPerSecond - input.omegaRadiansPerSecond) < Math.abs(cached.omegaRadiansPerSecond - input.omegaRadiansPerSecond)) {
            cached.omegaRadiansPerSecond = prev.omegaRadiansPerSecond;
        }
        cached.omegaRadiansPerSecond = math_utils.move_toward(cached.omegaRadiansPerSecond, input.omegaRadiansPerSecond, omega_accel_limit);
        input.omegaRadiansPerSecond = cached.omegaRadiansPerSecond;
    }

    public void calculate(ChassisSpeeds input, ChassisSpeeds prev) {
        calculate(input, prev, strafe_accel_limit);
    }
}
