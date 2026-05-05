package frc.robot.utils.controls;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class profiled_pid implements Sendable {

    private final ProfiledPIDController pid;
    private final SimpleMotorFeedforward ff;
    private State goal = new State();
    double prev_velocity = 0;

    public profiled_pid(pid_config config, double dts) {
        pid = new ProfiledPIDController(config.kP * config.kGain, config.kI * config.kGain, config.kD * config.kGain, config.constraints, dts);
        ff = new SimpleMotorFeedforward(0, config.kV, config.kA, dts);
        if(config.is_continuous) {
            pid.enableContinuousInput(config.min_input, config.max_input);
        }
        pid.setIntegratorRange(config.min_integral, config.max_integral);
    }

    public void reset(double position, double velocity) {
        pid.reset(position, velocity);
        prev_velocity = velocity;
    }

    public double calculate(double measurement, double target_position, double target_vel) {
        goal.position = target_position;
        goal.velocity = target_vel;
        double feedback = pid.calculate(measurement, goal);
        double velocity = pid.getSetpoint().velocity;
        double feedforward = ff.calculateWithVelocities(prev_velocity, velocity);
        prev_velocity = velocity;
        return feedforward + feedback;
    }

    public State get_state() {
        return pid.getSetpoint();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        pid.initSendable(builder);
        // builder.addDoubleProperty("kV", () -> ff.getKv(), (kV) -> { this.kV = kV; });
        // builder.addDoubleProperty("kA", () -> kA, (kA) -> { this.kA = kA; });
    }

}
