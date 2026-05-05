// modified WPILIB SlewRateLimiter class to play nice with chassis speeds

package frc.robot.utils.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.utils.math_utils;
import frc.robot.utils.swerve.swerve_kin2.chassis_output;

public class swerve_slew {

    private final double strafe_rate_limit; // max change per calculate call
    private final double omega_rate_limit; // max change per calculate call

    private double prev_x, prev_y, prev_omega;

    public swerve_slew(double strafe_rate_limit_s, double omega_rate_limit_s, double control_dts) {
        this.strafe_rate_limit = strafe_rate_limit_s * control_dts;
        this.omega_rate_limit = omega_rate_limit_s * control_dts;

        prev_x = 0;
        prev_y = 0;
        prev_omega = 0;
    }

    // the orbit thing
    public void calculate(chassis_output input) {
        double dx = input.x_output - prev_x;
        double dy = input.y_output - prev_y;
        double delta_norm = math_utils.hypot(dx, dy);
        
        double change = MathUtil.clamp(
            delta_norm,
            0,
            strafe_rate_limit);

        prev_x += dx * change;
        prev_y += dy * change;

        double delta_omega = input.omega_output - prev_omega;
        prev_omega += math_utils.clamp(-omega_rate_limit, omega_rate_limit, delta_omega);

        input.x_output = prev_x;
        input.y_output = prev_y;
        input.omega_output = prev_omega;
    }

    public void reset(double x, double y, double omega) {
        prev_x = x;
        prev_y = y;
        prev_omega = omega;
    }
}
