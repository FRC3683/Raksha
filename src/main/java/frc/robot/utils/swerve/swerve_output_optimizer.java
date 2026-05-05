package frc.robot.utils.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.utils.math_utils;
import frc.robot.utils.swerve.swerve_kin2.chassis_output;

/**
 * slip angle feels important....
 * use the change in wheel direction to decelerate, using "slip angle" or smth idk how it works tbh
 * "carry momentum through the turn" I think that's what Zaeem said at least
 * hopefully this lets us change direction more efficiently = faster/more responsive swerve
 * how this affects tread wear? idk i'm not the one who has to change them lets go FAST
 */
public class swerve_output_optimizer {


    private final double free_speed_mps;
    private final Angle max_slip_angle;
    private final chassis_output target_cache = new chassis_output();
    private final chassis_output output_cache = new chassis_output();
    private final ChassisSpeeds speeds = new ChassisSpeeds();

    public swerve_output_optimizer(LinearVelocity free_speed, Angle max_slip_angle, double control_dts) {
        free_speed_mps = free_speed.in(MetersPerSecond);
        this.max_slip_angle = max_slip_angle;
    }

    // private double calculate_voltage_from_torque(double output_torque, double current_speed_mps) {
    //     return (output_torque/stall_torque_Nm) + (current_speed_mps/free_speed_mps);
    // }

    // private double torque_from_accel(double accel){
    //     double torque = robot_mass_kg * accel * wheel_radius_m * drive_ratio; // TODO or divide by drive ratio?
    //     return torque;
    // }

    public chassis_output optimize(ChassisSpeeds current_speeds, chassis_output target_output) {
        return optimize(current_speeds, target_output, 1.0, max_slip_angle);
    }

    public chassis_output optimize(ChassisSpeeds current_speeds, chassis_output target_output, 
            double torque_limit, Angle slip_angle_limit) {

        target_cache.clone(target_output);
        math_utils.copy(current_speeds, speeds);

        torque_limit = MathUtil.clamp(torque_limit, 0.05, 1);
        slip_angle_limit = Radians.of(MathUtil.clamp(slip_angle_limit.in(Radians), 0, max_slip_angle.in(Radians)));

        if(math_utils.is_strafe_zero(speeds, 0.03)) {
            target_cache.limit(torque_limit);
            output_cache.clone(target_cache);
            return output_cache;
        }

        if(target_cache.is_strafe_zero(0.03)) {
            target_cache.x_output = 0;
            target_cache.y_output = 0;
            double output_min = -torque_limit + (math_utils.hypot(speeds)/free_speed_mps); // torque_limit = torque_Nm / stall_torque_Nm
            if(output_min > 0) {
                var optimized = new chassis_output(speeds).hat().times(output_min, 0);
                target_cache.x_output = optimized.x_output;
                target_cache.y_output = optimized.y_output;
            }
            output_cache.clone(target_cache);
            return output_cache;
        }

        double current_speeds_magnitude = MathUtil.clamp(math_utils.hypot(speeds), 0, free_speed_mps);
        var current_speeds_theta = math_utils.trans(speeds).getAngle();
        var target_cache_magnitude = target_cache.magnitude();
        var target_cache_theta = target_cache.trans().getAngle();
        var delta_theta = target_cache_theta.minus(current_speeds_theta);

        // TODO maybe we want to follow arc to preserve momentum with a delta >90 degrees?
        double current_speeds_z = (Math.abs(delta_theta.getRadians()) > (Math.PI / 2.0)) ? -current_speeds_magnitude : current_speeds_magnitude;

        double output_max = torque_limit + (current_speeds_z / free_speed_mps); // torque_limit = torque_Nm / stall_torque_Nm

        target_cache_magnitude = MathUtil.clamp(target_cache_magnitude, 0, output_max);

        // TODO replace linear unlerp with cosine or something? maybe? idk what's optimal
        double k = math_utils.unlerp(0, free_speed_mps, current_speeds_magnitude);
        double slip_angle_limit_rad = math_utils.lerp(Math.PI / 2, slip_angle_limit.in(Radians), k);

        var slip_target = delta_theta.minus(current_speeds_z < 0 ? Rotation2d.k180deg : Rotation2d.kZero).getRadians();
        if(Math.abs(slip_target) > slip_angle_limit_rad) {
            slip_target = slip_angle_limit_rad * Math.signum(slip_target);
        }

        var output_theta = current_speeds_theta.minus(current_speeds_z < 0 ? Rotation2d.k180deg : Rotation2d.kZero).plus(Rotation2d.fromRadians(slip_target));

        var optimized_output = new Translation2d(target_cache_magnitude, output_theta);

        // TODO torque limit omega?

        output_cache.x_output = optimized_output.getX();
        output_cache.y_output = optimized_output.getY();
        output_cache.omega_output = target_cache.omega_output;
        return output_cache;
    }
}
