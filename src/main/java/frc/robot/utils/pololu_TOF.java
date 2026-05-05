package frc.robot.utils;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Millimeters;

import java.util.Optional;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// https://www.pololu.com/category/283/pololu-digital-distance-sensors
// https://www.pololu.com/product/5472

public class pololu_TOF extends DigitalInput {

    public enum type {
        short_range_og_50cm     ( 50, 3, 4, 1, 50, 900, 1900 ),
        short_range_v2_50cm     ( 142, 1, 1.33, 0.1, 50, 900, 1900 ),
        long_range_130_cm       ( 100, 1, 0.5, 4, 130, 900, 1900 ),
        long_range_300_cm       ( 30, 2, 0.5, 4, 300, 900, 1900 ),
        ;

        int min_update_rate_hz;
        double resolution;
        Distance min_range, max_range;
        int detected_min_us, detected_max_us;

        type(int min_update_rate_hz, double resolution_mm, double resolution_us, double min_range_cm, double max_range_cm, int detected_min_us, int detected_max_us) {
            this.min_update_rate_hz = min_update_rate_hz;
            resolution = resolution_mm / resolution_us;
            min_range = Centimeters.of(min_range_cm);
            max_range = Centimeters.of(max_range_cm);
            this.detected_min_us = detected_min_us;
            this.detected_max_us = detected_max_us;
        }
    }

    private final DutyCycle duty_cycle;
    private final type type;

    public pololu_TOF(int channel, type type) {
        super(channel);
        duty_cycle = new DutyCycle(this);
        this.type = type;
    }

    public void dash() {
        SmartDashboard.putNumber("pololu_" + getChannel(), duty_cycle.getHighTimeNanoseconds());
    }

    public boolean is_connected() {
        return duty_cycle.getFrequency() > type.min_update_rate_hz;
    }

    private int high_time_us() {
        return duty_cycle.getHighTimeNanoseconds() / 1000;
    }

    @Override
    public boolean get() {
        if(RobotBase.isSimulation()) {
            return super.get();
        }
        int high_time_us = high_time_us();
        return high_time_us > type.detected_min_us && high_time_us < type.detected_max_us;
    }

    public Optional<Distance> get_distance() {
        if(RobotBase.isSimulation()) {
            return Optional.empty();
        }
        double high_time_us = high_time_us();
        if( high_time_us > type.detected_max_us || high_time_us < type.detected_min_us ) {
            return Optional.empty();
        }
        var distance = Millimeters.of( type.resolution * (high_time_us - 1000) );
        return Optional.of( distance);
    }
}
