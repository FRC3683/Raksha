package frc.robot.utils;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class dio_srx_mag_encoder extends DutyCycleEncoder {

    public dio_srx_mag_encoder(int channel, Angle abs_offset) {
        super( channel, 1.0, abs_offset.in(Rotations) );
        setDutyCycleRange(1.0 / 4096, 4095.0 / 4096);
    }

    public Angle get_abs_raw() {
        return Rotations.of(super.get());
    }

    public Angle get_abs() {
        return get_abs_raw();
    }
}
