package frc.robot.utils.swerve;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.math_utils;
import frc.robot.utils.swerve.swerve_kin2.chassis_output;

public final class torque_limiting {
    private torque_limiting() {}

    public static double[] estimate_acceleration( chassis_output applied_out, ChassisSpeeds current_speeds, double free_speed) {
        double deccel_factor = 0.9;
        double accel_factor = 1.12;
        double torque_y = applied_out.y_output - (current_speeds.vyMetersPerSecond/free_speed);
        double torque_x = applied_out.x_output - (current_speeds.vxMetersPerSecond/free_speed);
        // double torque = output - (speed/free_speed); 
        double[] torque_vals = {torque_x, torque_y};

        if (torque_vals[0] < 0 && current_speeds.vyMetersPerSecond > 0 || torque_vals[0] > 0 && current_speeds.vyMetersPerSecond < 0){
            torque_vals[0] *= deccel_factor;
        } else {
            torque_vals[0] *= accel_factor;
        };


         if (torque_vals[1] < 0 && current_speeds.vyMetersPerSecond > 0 || torque_vals[1] > 0 && current_speeds.vyMetersPerSecond < 0){
            torque_vals[1] *= deccel_factor;
        } else {
            torque_vals[1] *= accel_factor;
        };
        
        // if (torque_vals[0]>0){
        //     torque_vals[0]*= scale_factor;
        // };

        // if (torque_vals[1] < 0){
        //     torque_vals[1] *= -scale_factor;
        // };

        // if (torque_vals[1]>0){
        //     torque_vals[1]*= scale_factor;
        // };
        return torque_vals;
    }

    // public static double acc_comparator();
    // scaling factor to account for the friction as it will help the robot slow down faster, but make it more difficult to accelerate
    // public static void scale_friction(){
    //     double[] torque_vals = estimate_torque(null, null, 0);

    //     double scale_factor = 1.12;

    //     if (torque_vals[0] < 0){
    //         torque_vals[0] *= -scale_factor;
    //     };

    //     if (torque_vals[0]>0){
    //         torque_vals[0]*= scale_factor;
    //     };

    //     if (torque_vals[1] < 0){
    //         torque_vals[1] *= -scale_factor;
    //     };

    //     if (torque_vals[1]>0){
    //         torque_vals[1]*= scale_factor;
    //     };
        
    // };
    public static double limit_torque( double target_output, double current_speed, double free_speed, double torque_limit ) {
        double neutral = ( current_speed / free_speed );
        return MathUtil.clamp( target_output, neutral - torque_limit, neutral + torque_limit );
    }

    public static double limit_torque_asym( double target_output, double current_speed, double free_speed, double accel_limit, double decel_limit ) {
        double neutral = current_speed / free_speed;
        double torque_limit = Math.abs( target_output ) > Math.abs( neutral ) ? accel_limit : decel_limit;
        return MathUtil.clamp( target_output, neutral - torque_limit, neutral + torque_limit );
    }

    public static chassis_output limit_torque( chassis_output target_output, ChassisSpeeds current_speeds, double free_speed, double torque_limit, Angle max_slip_angle ) {
        if( math_utils.hypot( current_speeds ) < 0.05 ) {
            double l = limit_torque( target_output.magnitude(), 0, free_speed, torque_limit );
            return target_output.copy().hat().times( l, 1.0 );
        }

        if( target_output.magnitude_squared() < 0.001 ) {
            double l = limit_torque( 0, math_utils.hypot ( current_speeds ), free_speed, torque_limit );
            var out = new Translation2d( l, new Translation2d( current_speeds.vxMetersPerSecond, current_speeds.vyMetersPerSecond ).getAngle() );
            SmartDashboard.putNumberArray("huh", new double[]{ out.getX(), out.getY(), out.getNorm() });
            target_output.x_output = out.getX();
            target_output.y_output = out.getY();
            SmartDashboard.putNumberArray("huh2", new double[]{ target_output.x_output, target_output.y_output });

            SmartDashboard.putNumber("rand", l);
            SmartDashboard.putNumber("rand2", out.getNorm());

            return new chassis_output( out.getX(), out.getY(), target_output.omega_output );
        }

        max_slip_angle = max_slip_angle.plus( Degrees.of( ( 1 - ( math_utils.hypot( current_speeds ) / free_speed ) ) * 20 ) );
        double dot = math_utils.dot( target_output.hat().trans(), math_utils.hat( math_utils.trans( current_speeds ) ) );
        if( dot > -0.45 ) {
            double l = limit_torque( target_output.magnitude(), math_utils.hypot( current_speeds ), free_speed, torque_limit );
            Translation2d t = math_utils.rotate_towards( math_utils.trans( current_speeds ), target_output.trans(), max_slip_angle );
            t = math_utils.hat( t ).times( l );
            return new chassis_output( t.getX(), t.getY(), target_output.omega_output );
        }
        double l = limit_torque( -target_output.magnitude(), math_utils.hypot( current_speeds ), free_speed, torque_limit );
        Translation2d t = math_utils.rotate_towards( math_utils.trans( current_speeds ), target_output.trans().unaryMinus(), max_slip_angle );
        t = math_utils.hat( t ).times( l );
        return new chassis_output( t.getX(), t.getY(), target_output.omega_output );
    }
}
