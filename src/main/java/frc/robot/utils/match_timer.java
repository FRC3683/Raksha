package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.robot;

public class match_timer {
    public enum phase {
        AUTO,
        LIL_BIT,
        TRANSITION,
        SHIFT_1,
        SHIFT_2,
        SHIFT_3,
        SHIFT_4,
        ENDGAME
    }

    public static final double match_start = 0;
    public static final double auto_end = 20;
    public static final double transition_start = auto_end + 3;
    public static final double transition_end = transition_start + 10;
    public static final double shift_1_end = transition_end + 25;
    public static final double shift_2_end = shift_1_end + 25;
    public static final double shift_3_end = shift_2_end + 25;
    public static final double shift_4_end = shift_3_end + 25;
    public static final double end_of_match = shift_4_end + 30;

    private static final double preview_time = 6;

    private static final Timer auto_timer = new Timer();
    private static final Timer tele_timer = new Timer();
    private static boolean is_auto = false;

    public static void auto_init() {
        auto_timer.restart();
        is_auto = true;
    }

    public static void tele_init() {
        tele_timer.restart();
        is_auto = false;
    }

    public static double match_seconds_elapsed() {
        if( is_auto ) {
            return auto_timer.get();
        }
        return tele_timer.get() + 23;
    }

    public static double seconds_left_in_shift() {
        double elapsed = match_seconds_elapsed();
        phase p = get_phase( elapsed );

        if( p == phase.AUTO || p == phase.LIL_BIT ) {
            return auto_end - elapsed;
        }

        if( p == phase.TRANSITION ) {
            return transition_end - elapsed;
        }

        if( p == phase.SHIFT_1 ) {
            return shift_1_end - elapsed;
        }
        if( p == phase.SHIFT_2 ) {
            return shift_2_end - elapsed;
        }
        if( p == phase.SHIFT_3 ) {
            return shift_3_end - elapsed;
        }
        if( p == phase.SHIFT_4 ) {
            return shift_4_end - elapsed;
        }

        return end_of_match - elapsed;
    }

    public static phase get_phase( double elapsed ) {
        if( elapsed < auto_end ) {
            return phase.AUTO;
        }
        if( elapsed < transition_start ) {
            return phase.LIL_BIT;
        }
        if( elapsed < transition_end ) {
            return phase.TRANSITION;
        }

        if( elapsed < shift_1_end ) {
            return phase.SHIFT_1;
        }
        if( elapsed < shift_2_end ) {
            return phase.SHIFT_2;
        }
        if( elapsed < shift_3_end ) {
            return phase.SHIFT_3;
        }
        if( elapsed < shift_4_end ) {
            return phase.SHIFT_4;
        }

        return phase.ENDGAME;
    }

    public static phase get_current_phase() {
        return get_phase( match_seconds_elapsed() );
    }

    public static boolean is_hub_active_raw( double match_elapsed ) {
        if( match_elapsed < transition_end || match_elapsed > shift_4_end ) {
            return true;
        }

        String match_data = DriverStation.getGameSpecificMessage();
        if( match_data == null || match_data.isEmpty() ) {
            return true;
        }

        boolean red_won = match_data.charAt( 0 ) == 'R';
        boolean dave_won = red_won == robot.is_red();

        phase p = get_phase( match_elapsed );

        if( dave_won ) {
            return p == phase.SHIFT_2 || p == phase.SHIFT_4;
        }
        return p == phase.SHIFT_1 || p == phase.SHIFT_3;
    }

    public static boolean is_hub_active( double match_elapsed ) {
        return is_hub_active_raw( match_elapsed ) || is_hub_active_raw( match_elapsed + 1.5 ) || is_hub_active_raw( match_elapsed - 1.5 );
    }

    public static boolean is_hub_active() {
        double elapsed = match_seconds_elapsed();
        return is_hub_active( elapsed );
    }

    public static boolean hub_active_preview( double lookahead ) {
        return is_hub_active( match_seconds_elapsed() + lookahead );
    }

    public static boolean hub_active_preview() {
        return is_hub_active( match_seconds_elapsed() + preview_time );
    }

}
