package frc.robot.utils.controls;

import edu.wpi.first.math.controller.PIDController;

public class pid extends PIDController {
    public pid(pid_config config, double dts) {
        super(config.kP * config.kGain, config.kI * config.kGain, config.kD * config.kGain, dts);
        if(config.is_continuous) {
            enableContinuousInput(config.min_input, config.max_input);
        }
        setIntegratorRange(config.min_integral, config.max_integral);
    }
}
