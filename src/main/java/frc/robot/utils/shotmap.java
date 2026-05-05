package frc.robot.utils;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;

public class shotmap {

    public static class shot implements Interpolatable<shot> {
        public double vx, vz;
        public boolean valid = false;

        public shot() {}

        @Override
        public shot interpolate( shot end_value, double t ) {
            shot s = new shot();
            s.vx = math_utils.lerp( vx, end_value.vx, t );
            s.vz = math_utils.lerp( vz, end_value.vz, t );
            s.valid = true;
            return s;
        }
    }

    private final InterpolatingTreeMap<Double, shot> map = new InterpolatingTreeMap<>( InverseInterpolator.forDouble(), shot::interpolate );

    private double min_key = Double.MAX_VALUE, max_key = 0;

    public shotmap( double rpm_trim, double[] shots ) {

        for( int i = 0; i <= shots.length - 3; i += 3 ) {
            double inches = shots[ i ];
            double deg = shots[ i + 1 ];
            double rpm = shots[ i + 2 ] + rpm_trim;

            double rps = rpm / 60.0;
            double radps = Units.rotationsToRadians( rps );

            double exit_mps = radps * Units.inchesToMeters(2); // flywheel radius

            Translation2d exit = new Translation2d( exit_mps, Rotation2d.fromDegrees( deg ) );

            shot s = new shot();
            s.vx = exit.getX();
            s.vz = exit.getY();
            double meters = Units.inchesToMeters(inches);
            map.put( meters, s );

            if( meters < min_key ) {
                min_key = meters;
            }
            if( meters > max_key ) {
                max_key = meters;
            }
        }
    }

    public boolean has_shot( double distance ) {
        return min_key <= distance && distance <= max_key;
    }

    public shot get_shot( double distance ) {
        boolean valid = has_shot( distance );
        distance = MathUtil.clamp( distance, min_key+0.01, max_key-0.01 );
        shot s = map.get( distance );
        s.valid = valid;
        return s;
    }
}