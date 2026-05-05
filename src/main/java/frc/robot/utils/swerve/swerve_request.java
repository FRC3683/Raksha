package frc.robot.utils.swerve;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc.robot.utils.swerve.swerve_kin2.chassis_output;

public class swerve_request {
    public final chassis_output target_field_relative = new chassis_output();
    public Angle max_slip_angle = Degrees.of( 18 );
    public double max_strafe_torque = 1.0;
    public double max_omega_torque = 1.0;
    public double turn_bias = 0.7;
    public boolean form_x_when_stopped = false;
    public boolean stop_motors = false;

    public static swerve_request form_x() {
        return new swerve_request().with_form_x_when_stopped(true);
    }

    public static swerve_request static_brake() {
        return new swerve_request().with_stop_motors(true);
    }

    public swerve_request with_chassis_output(double x_output, double y_output, double omega_output) {
        target_field_relative.x_output = x_output;
        target_field_relative.y_output = y_output;
        target_field_relative.omega_output = omega_output;
        return this;
    }

    public swerve_request with_max_slip_angle(Angle max_slip_angle) {
        this.max_slip_angle = max_slip_angle;
        return this;
    }

    public swerve_request with_turn_bias(double turn_bias) {
        this.turn_bias = turn_bias;
        return this;
    }

    public swerve_request with_max_torque(double max_strafe_torque, double max_omega_torque) {
        this.max_strafe_torque = max_strafe_torque;
        this.max_omega_torque = max_omega_torque;
        return this;
    }

    public swerve_request with_form_x_when_stopped(boolean form_x_when_stopped) {
        this.form_x_when_stopped = form_x_when_stopped;
        return this;
    }

    public swerve_request with_stop_motors(boolean stop_motors) {
        this.stop_motors = stop_motors;
        return this;
    }


}