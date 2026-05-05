package frc.robot.utils.swerve;

import static edu.wpi.first.units.Units.Radians;

import java.util.Arrays;
import java.util.Collections;
import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.Measure;
import frc.robot.constants;
import frc.robot.utils.math_utils;

public class swerve_kin2 extends SwerveDriveKinematics {
    public swerve_kin2(Translation2d[] module_mount_positions, Translation2d center_of_turning) {
        super(module_mount_positions);
        chassis_radius = Collections.max(Arrays.asList(module_mount_positions), (pos1, pos2) -> {
            return Double.compare(pos1.getNorm(), pos2.getNorm());
        }).getNorm();
        this.num_modules = module_mount_positions.length;
        this.module_mount_positions = module_mount_positions;
        center_matrices = new swerve_kin2_matrices(center_of_turning);
        forward_kin = center_matrices.inverse_kin.pseudoInverse();
    }

    public class swerve_kin2_matrices {
        public swerve_kin2_matrices(Translation2d cot_meters) {
            inverse_kin = new SimpleMatrix(num_modules * 2, 3);
            big_inverse_kin = new SimpleMatrix(num_modules * 2, 4);
            for (int i = 0; i < num_modules; i++) {
                inverse_kin.setRow(
                    i * 2, 0, /* Start Data */ 1, 0, -module_mount_positions[i].getY() + cot_meters.getY());
                inverse_kin.setRow(
                    i * 2 + 1,
                    0, /* Start Data */
                    0,
                    1,
                    +module_mount_positions[i].getX() - cot_meters.getX());
                big_inverse_kin.setRow(
                    i * 2,
                    0, /* Start Data */
                    1,
                    0,
                    -module_mount_positions[i].getX() + cot_meters.getX(),
                    -module_mount_positions[i].getY() + cot_meters.getY());
                big_inverse_kin.setRow(
                    i * 2 + 1,
                    0, /* Start Data */
                    0,
                    1,
                    -module_mount_positions[i].getY() + cot_meters.getY(),
                    +module_mount_positions[i].getX() - cot_meters.getX());
            }
        }

        private final SimpleMatrix inverse_kin;
        private final SimpleMatrix big_inverse_kin;
    }

    public static class module_output implements Comparable<module_output> {
        public double drive_output;
        public double theta_rad;
        public double omega_radps;

        public module_output(double output, double theta_rad, double omega_radps) {
            this.drive_output = output;
            this.theta_rad = theta_rad;
            this.omega_radps = omega_radps;
        }

        @Override
        public boolean equals(Object obj) {
            if(obj instanceof module_output) {
                module_output other = (module_output) obj;
                return Math.abs(drive_output - other.drive_output) <= 0.01
                    && Math.abs(theta_rad - other.theta_rad) <= Units.degreesToRadians(0.1)
                    && Math.abs(omega_radps - other.omega_radps) <= Units.degreesToRadians(0.1);
            }
            return false;
        }

        @Override
        public int compareTo(module_output other) {
            return Double.compare(drive_output, other.drive_output);
        }

        public module_output optimize(Angle current_position) {
            var theta = Rotation2d.fromRadians(theta_rad);
            var delta = theta.minus(Rotation2d.fromRadians(current_position.in(Radians)));
            if(Math.abs(delta.getRadians()) > (Math.PI / 2.0)) {
                drive_output *= -1;
                theta_rad = theta.rotateBy(Rotation2d.kPi).getRadians();
            }
            return this;
        }
    }

    public static class module_speed implements Comparable<module_speed> {
        public double drive_speed_mps;
        public double theta_rad;
        public double omega_radps;

        public module_speed(double speed_mps, double theta_rad, double omega_radps) {
            this.drive_speed_mps = speed_mps;
            this.theta_rad = theta_rad;
            this.omega_radps = omega_radps;
        }

        @Override
        public boolean equals(Object obj) {
            if(obj instanceof module_speed) {
                module_speed other = (module_speed) obj;
                return Math.abs(drive_speed_mps - other.drive_speed_mps) <= 0.001
                    && Math.abs(theta_rad - other.theta_rad) <= Units.degreesToRadians(0.1)
                    && Math.abs(omega_radps - other.omega_radps) <= Units.degreesToRadians(0.1);
            }
            return false;
        }

        @Override
        public int compareTo(module_speed other) {
            return Double.compare(drive_speed_mps, other.drive_speed_mps);
        }
    }

    public static class chassis_output {
        public double x_output = 0, y_output = 0, omega_output = 0;

        public chassis_output(double x_output, double y_output, double omega_output) {
            this.x_output = x_output;
            this.y_output = y_output;
            this.omega_output = omega_output;
        }

        public chassis_output(double x_output, double y_output) {
            this(x_output, y_output, 0);
        }

        public chassis_output(ChassisSpeeds output) {
            this(output.vxMetersPerSecond, output.vyMetersPerSecond, output.omegaRadiansPerSecond);
        }

        public chassis_output() {
            this(0, 0, 0);
        }

        public ChassisSpeeds as_speeds() {
            return new ChassisSpeeds(x_output, y_output, omega_output);
        }

        public chassis_output plus(Translation2d other) {
            return new chassis_output(x_output + other.getX(), y_output + other.getY(), omega_output);
        }

        public chassis_output deadbanded(double strafe_deadzone, double turn_deadzone) {
            if(magnitude_squared() < strafe_deadzone * strafe_deadzone) {
                x_output = 0;
                y_output = 0;
            }
            if(Math.abs(omega_output) < turn_deadzone) {
                omega_output = 0;
            }
            return this;
        }

        public chassis_output to_field_relative(Rotation2d robot_angle) {
            var rotated = new Translation2d(x_output, y_output).rotateBy(robot_angle);
            x_output = rotated.getX();
            y_output = rotated.getY();
            return this;
        }

        public boolean is_strafe_zero(double deadband) {
            return (x_output * x_output) + (y_output * y_output) < (deadband * deadband);
        }

        public chassis_output to_robot_relative(Rotation2d robot_angle) {
            return to_field_relative(robot_angle.unaryMinus());
        }

        public chassis_output clone(chassis_output other) {
            x_output = other.x_output;
            y_output = other.y_output;
            omega_output = other.omega_output;
            return this;
        }

        public void move_towards(double target_x_output, double target_y_output, double max_delta) {
            double dx = target_x_output - x_output;
            double dy = target_y_output - y_output;
            var hypot_sq = math_utils.hypot_squred(dx, dy);
            if(hypot_sq < max_delta * max_delta) {
                x_output = target_x_output;
                y_output = target_y_output;
            } else {
                var hypot = Math.sqrt(hypot_sq);
                x_output += (dx / hypot) * max_delta;
                y_output += (dy / hypot) * max_delta;
            }
        }

        public void move_towards(double target_x_output, double target_y_output, double target_omega_output, double max_delta) {
            move_towards(target_x_output, target_y_output, max_delta);
            omega_output = math_utils.move_toward(omega_output, target_omega_output, max_delta);
        }

        public chassis_output copy() {
            return new chassis_output(x_output, y_output, omega_output);
        }

        public Translation2d trans() {
            return new Translation2d(x_output, y_output);
        }

        public double magnitude() {
            return math_utils.hypot(x_output, y_output);
        }

        public double magnitude_squared() {
            return math_utils.hypot_squred(x_output, y_output);
        }

        public chassis_output hat() {
            if(x_output == 0 && y_output == 0) {
                // prevents divide by 0 error
                return this;
            }
            double magnitude = magnitude();
            x_output /= magnitude;
            y_output /= magnitude;
            return this;
        }

        public chassis_output times(double strafe_k, double omega_k) {
            x_output *= strafe_k;
            y_output *= strafe_k;
            omega_output *= omega_k;
            return this;
        }

        public chassis_output limit(double max_magnitude) {
            double magnitude = magnitude();
            if(magnitude > max_magnitude) {
                x_output *= (max_magnitude / magnitude);
                y_output *= (max_magnitude / magnitude);
            }
            return this;
        }
    }

    public void desaturateWheelSpeeds(module_output[] module_states) {
        double target = 0;
        for(int i = 0; i < module_states.length; ++i) {
            target = Math.max(target, Math.abs(module_states[i].drive_output));
        }
        if(target == 0) {
            return;
        }

        if(target > max_output) {
            for(var state : module_states) {
                state.drive_output *= (max_output / target);
            }
        }
    }

    /**
     * clamp the chassis_output to the given max strafe and max turn output
     * turn bias 0.5 is default, clamp strafing and turning equally
     * turn bias 0 is do no turning while strafing at max output
     * turn bias 1 is do no strafing while turning at max output
     */
    public chassis_output clamp(chassis_output output, double turn_bias) {
        // ensure valid bias
        turn_bias = math_utils.clamp01(turn_bias);

        // first clamp strafing and turning individually, so we have each achievable on their own.
        double len = math_utils.hypot(output);
        if(len > max_output) {
            double k = max_output / len;
            output.x_output *= k;
            output.y_output *= k;
        }
        output.omega_output = math_utils.clamp(-max_output, max_output, output.omega_output);

        // fun stuff starts here
        double strafe_k = Math.abs(math_utils.hypot(output) / max_output); // 0...1
        double turn_k = Math.abs(output.omega_output / max_output); // 0...1
        double k = (strafe_k + turn_k); // 0...2
        final double max_k = 1.0;
        if(k > max_k) { // saturating speeds, need to compensate


            // thought this would play well with heading snap tracking a moving heading, no noticable improvement in sim...
            // also it would feel wierd to drive i think.... idk tho
            // if(turn_k <= turn_bias) {
                // double adjusted_strafe_k = max_k - turn_k;
                // output.x_output *= adjusted_strafe_k / strafe_k;
                // output.y_output *= adjusted_strafe_k / strafe_k;
                // return output;
            // }


            // limit strafing and turning differently based on bias.
            double max_strafe_k = max_k - (turn_k * turn_bias);
            double adjusted_strafe_k = strafe_k * max_strafe_k;
            double adjusted_turn_k = max_k - adjusted_strafe_k;

            if(strafe_k != 0) {
                output.x_output *= adjusted_strafe_k / strafe_k;
                output.y_output *= adjusted_strafe_k / strafe_k;
            }
            if(turn_k != 0) {
                output.omega_output *= adjusted_turn_k / turn_k;
            }
        }

        return output;
    }

    public void to_module_states(chassis_output chassis_speeds, swerve_kin2_matrices matrices, module_output[] module_states) {
        if(chassis_speeds.x_output == 0.0
                && chassis_speeds.y_output == 0.0
                && chassis_speeds.omega_output == 0.0) {
            for (int i = 0; i < num_modules; i++) {
                module_states[i].drive_output = 0.0;
            }
            return;
        }

        var chassis_speeds_vec = new SimpleMatrix(3, 1);
        chassis_speeds_vec.setColumn(
                0,
                0,
                chassis_speeds.x_output,
                chassis_speeds.y_output,
                chassis_speeds.omega_output / chassis_radius);

        var module_velocity_mat = matrices.inverse_kin.mult(chassis_speeds_vec);

        var acceleration_vec = new SimpleMatrix(4, 1);
        acceleration_vec.setColumn(0, 0, 0, 0, Math.pow(chassis_speeds.omega_output / chassis_radius, 2), 0);

        var module_accel_states_mat = matrices.big_inverse_kin.mult(acceleration_vec);

        for (int i = 0; i < num_modules; i++) {
            double x = module_velocity_mat.get(i * 2, 0);
            double y = module_velocity_mat.get(i * 2 + 1, 0);

            double ax = module_accel_states_mat.get(i * 2, 0);
            double ay = module_accel_states_mat.get(i * 2 + 1, 0);

            double speed = Math.hypot(x, y);
            Rotation2d angle = new Rotation2d(x, y);

            var trig_theta_angle_mat = new SimpleMatrix(2, 2);
            trig_theta_angle_mat.setColumn(0, 0, angle.getCos(), -angle.getSin());
            trig_theta_angle_mat.setColumn(1, 0, angle.getSin(), angle.getCos());

            var accel_vec = new SimpleMatrix(2, 1);
            accel_vec.setColumn(0, 0, ax, ay);

            var omegaVector = trig_theta_angle_mat.mult(accel_vec);

            double omega = (omegaVector.get(1, 0) / (speed)) - (chassis_speeds.omega_output / chassis_radius);
            module_states[i].drive_output = speed;
            module_states[i].theta_rad = angle.getRadians();
            module_states[i].omega_radps = omega * constants.swerve.max_module_speed_mps;
        }
    }

    public static boolean is_stopped(module_output[] states) {
        for(var state : states) {
            if(Math.abs(state.drive_output) > 0.005) {
                return false;
            }
        }
        return true;
    }

    public module_output[] form_x() {
        var states = new module_output[num_modules];
        for(int i = 0; i < states.length; ++i) {
            states[i] = new module_output(0, module_mount_positions[i].getAngle().getRadians(), 0);
        }
        return states;
    }

    public void to_module_states(chassis_output output, module_output[] module_states) {
        to_module_states(output, center_matrices, module_states);
    }

    public chassis_output to_chassis_output(module_output... module_states) {
        if (module_states.length != num_modules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of wheel locations provided in "
                            + "constructor");
        }
        var module_states_mat = new SimpleMatrix(num_modules * 2, 1);

        for (int i = 0; i < num_modules; i++) {
            var module_state = module_states[i];
            module_states_mat.set(i * 2, 0, module_state.drive_output * Math.cos(module_state.theta_rad));
            module_states_mat.set(i * 2 + 1, module_state.drive_output * Math.sin(module_state.theta_rad));
        }

        var chassisSpeedsVector = forward_kin.mult(module_states_mat);
        return new chassis_output(
                chassisSpeedsVector.get(0, 0),
                chassisSpeedsVector.get(1, 0),
                chassisSpeedsVector.get(2, 0));
    }

    public ChassisSpeeds to_chassis_speeds(module_speed... module_speeds) {
        if (module_speeds.length != num_modules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of wheel locations provided in "
                            + "constructor");
        }
        var module_states_mat = new SimpleMatrix(num_modules * 2, 1);

        for (int i = 0; i < num_modules; i++) {
            var module_state = module_speeds[i];
            module_states_mat.set(i * 2, 0, module_state.drive_speed_mps * Math.cos(module_state.theta_rad));
            module_states_mat.set(i * 2 + 1, module_state.drive_speed_mps * Math.sin(module_state.theta_rad));
        }

        var chassisSpeedsVector = forward_kin.mult(module_states_mat);
        return new ChassisSpeeds(
                chassisSpeedsVector.get(0, 0),
                chassisSpeedsVector.get(1, 0),
                chassisSpeedsVector.get(2, 0));
    }


    public static final double max_output = 1.0;

    public final int num_modules;
    public final Translation2d[] module_mount_positions;
    public final double chassis_radius;

    private final swerve_kin2_matrices center_matrices;
    private final SimpleMatrix forward_kin;
}
