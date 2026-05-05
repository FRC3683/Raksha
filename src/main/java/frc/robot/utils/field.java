package frc.robot.utils;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;


public record field( Translation2d hub_position, Translation2d pass_low_y, Translation2d pass_high_y, zone alliance_zone, zone nuetral_zone, zone opp_zone ) {
    public static record zone( Rectangle2d include_zone, Rectangle2d[] exclude ) {
        public boolean in_zone( Translation2d pose ) {
            for( var e : exclude ) {
                if( e.contains( pose ) ) {
                    return false;
                }
            }
            return include_zone.contains( pose );
        }
    }

    public static record disconnected_zone( Rectangle2d[] zones ) {
        public boolean in_zone( Translation2d pose ) {
            for( var e : zones ) {
                if( e.contains( pose ) ) {
                    return true;
                }
            }
            return false;
        }
    }
}
