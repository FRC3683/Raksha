package frc.robot.utils.controls;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class pid_config {
    public double kP = 1.0, kI = 0.0, kD = 0.0, kGain = 1.0;
    public double kV = 0.0, kA = 0.0;
    public boolean is_continuous = false;
    public double min_input, max_input;
    public double min_integral = -1.0, max_integral = 1.0;
    public Constraints constraints = new Constraints(0, 0);

    public pid_config withKP(double kP) {
        this.kP = kP;
        return this;
    }

    public pid_config withKI(double kI) {
        this.kI = kI;
        return this;
    }

    public pid_config withKD(double kD) {
        this.kD = kD;
        return this;
    }

    public pid_config withGain(double kGain) {
        this.kGain = kGain;
        return this;
    }

    public pid_config with_feedforward(double kV, double kA) {
        this.kV = kV;
        this.kA = kA;
        return this;
    }

    public pid_config with_continuous_input(double min_input, double max_input) {
        is_continuous = true;
        this.min_input = min_input;
        this.max_input = max_input;
        return this;
    }

    public pid_config with_integral_range(double min_integral, double max_integral) {
        this.min_integral = min_integral;
        this.max_integral = max_integral;
        return this;
    }

    public pid_config with_trapezoid_constraints(double max_vel, double max_accel) {
        this.constraints = new Constraints(max_vel, max_accel);
        return this;
    }
}
