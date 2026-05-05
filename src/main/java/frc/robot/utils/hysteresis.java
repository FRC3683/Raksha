package frc.robot.utils;

import edu.wpi.first.math.MathUtil;

public class hysteresis {
    
    private boolean value;

    private double tight_threshold, loose_threshold;

    public hysteresis( double tight_threshold, double loose_threshold, boolean initial_value ) {
        this.tight_threshold = tight_threshold;
        this.loose_threshold = loose_threshold;
        value = initial_value;
    }

    public boolean get_value() {
        return value;
    }


    public hysteresis refresh( double measurement, double setpoint ) {
        double error = setpoint - measurement;
        if( value ) {
            value = MathUtil.isNear( 0, error, loose_threshold );
        } else {
            value = MathUtil.isNear( 0, error, tight_threshold );
        }
        return this;
    }

    public hysteresis bangbang_refresh( double measurement, double setpoint ) {
        double error = setpoint - measurement;
        if( value ) {
            value = MathUtil.isNear(0, error, loose_threshold );
        } else {
            value = MathUtil.isNear( 0, error, tight_threshold ) && measurement > setpoint;
        }
        return this;
    }


}
