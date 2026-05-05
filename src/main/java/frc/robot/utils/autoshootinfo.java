// package frc.robot.utils;

// import java.time.LocalTime;
// import java.time.temporal.TemporalField;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.utils.auto_selector.auto;

// public class autoshootinfo {

//     // public static LocalTime currentTime;
//     public static LocalTime auto_end;
//     public static LocalTime first_shift_start;
//     public static LocalTime second_shift_start;
//     public static LocalTime third_shift_start;
//     public static LocalTime fourth_shift_start;
//     public static LocalTime endgame_start;
//     public static LocalTime end_of_match;
//     public static boolean IsGoingFirst;
//     public static boolean hub_active = false;
//     public static boolean hub_active_preview = false;
//     public static boolean hub_active_preview_4 = false;
//     public static boolean hub_active_preview_1_5 = false;
//     // NOTE: BE SURE TO ADD THE FOLOWING LINES INTO autonomusinit FOR THE TIMER TO
//     // WORK:
//     // currentTime=LocalTime.now(); // get the time when the match starts
//     // autotimeshift=currentTime.plusSeconds(10); // auto
//     // secondshift=autotimeshift.plusSeconds(25); // first shift
//     // thirdshift=secondshift.plusSeconds(25); // second shift
//     // fourthshift=thirdshift.plusSeconds(25); // fouth shift

//     static {
//         autosetup();
//     }
    
//     public static String gameData = DriverStation.getGameSpecificMessage(); // sets up the variable to store who won
//                                                                             // auto

//     private static void setup( LocalTime currentTime ) {
//         SmartDashboard.putBoolean( "hub_active", false );
//         // SmartDashboard.putBoolean("AAA_Should Shoot?: ", true);
//         auto_end = currentTime.plusSeconds( 23 ); // auto
//         first_shift_start = auto_end.plusSeconds( 10 ); // first shift
//         second_shift_start = first_shift_start.plusSeconds( 25 ); // second shift
//         third_shift_start = second_shift_start.plusSeconds( 25 ); // third shift
//         fourth_shift_start = third_shift_start.plusSeconds( 25 ); // fouth shift
//         endgame_start = fourth_shift_start.plusSeconds( 25 );
//         end_of_match = endgame_start.plusSeconds( 30 );
//     }

//     public static void autosetup() {
//         LocalTime currentTime = LocalTime.now(); // get the time when the match starts
//         setup( currentTime );
//     }

//     public static void telesetup() {
//         LocalTime currentTime = LocalTime.now().minusSeconds( 23 ); // get the time when the match starts
//         setup( currentTime );
//     }

//     public static boolean is_endgame() {
//         LocalTime currentTime = LocalTime.now(); // get the time when the match starts
//         return currentTime.isAfter( endgame_start );
//     }

//     public static boolean is_transition() {
//         LocalTime currentTime = LocalTime.now(); // get the time when the match starts
//         return currentTime.isAfter( auto_end ) && currentTime.isBefore( first_shift_start );
//     }

//     // public static double shift_countdown_seconds() {
//     //     LocalTime currentTime = LocalTime.now(); // get the time when the match starts

//     //     if( currentTime.isAfter( endgame_start ) ) {
            
//     //         return end_of_match.minusSeconds( currentTime. ).getSecond();
//     //     } else if( currentTime.isAfter( fourth_shift_start ) ) {
//     //         return davecolour.equals( autowinner );
//     //     } else if( currentTime.isAfter( third_shift_start ) ) {
//     //         return !davecolour.equals( autowinner );
//     //     } else if( currentTime.isAfter( second_shift_start ) ) {
//     //         return davecolour.equals( autowinner );
//     //     } else if( currentTime.isAfter( first_shift_start ) ) {
//     //         return !davecolour.equals( autowinner );
//     //     } else {

//     //     }
//     // }

//     public static void shouldshootperiodic() {
//         gameData = DriverStation.getGameSpecificMessage();
//         LocalTime currentTime = LocalTime.now(); // get the time when the match starts
//         hub_active = ShouldShoot( currentTime );
//         hub_active_preview = ShouldShoot( currentTime.plusSeconds( 8 ) );
//         hub_active_preview_4 = ShouldShoot( currentTime.plusSeconds( 4 ) );
//         hub_active_preview_1_5 = ShouldShoot( currentTime.plusSeconds( 1 ) );
//         SmartDashboard.putBoolean( "hub_active", hub_active );
//         SmartDashboard.putNumber("AAA_CurrentTime: ", currentTime.getSecond());
//         SmartDashboard.putNumber("AAA_AutoShift: ", auto_end.getSecond());
//     }

//     public static boolean ShouldShoot( LocalTime currentTime ) {

//         if( currentTime.isBefore( first_shift_start ) || currentTime.isAfter( endgame_start ) ) {
//             return true;
//         }

//         if( gameData.isEmpty() ) {
//             return true;
//         }

//         String davecolour = DriverStation.getAlliance().orElse(Alliance.Red).toString();
//         davecolour = davecolour.substring(0, 1);
//         SmartDashboard.putString("AAA_Dave", davecolour);
//         String autowinner = gameData.substring(0, 1); // gets who won auto

//         if( currentTime.isAfter( fourth_shift_start ) ) {
//             return davecolour.equals( autowinner );
//         } else if( currentTime.isAfter( third_shift_start ) ) {
//             return !davecolour.equals( autowinner );
//         } else if( currentTime.isAfter( second_shift_start ) ) {
//             return davecolour.equals( autowinner );
//         } else if( currentTime.isAfter( first_shift_start ) ) {
//             return !davecolour.equals( autowinner );
//         }

//         return true;

//         // if (!gameData.isEmpty() && currentTime.isAfter(first_shift_start)) {
//         //     // return true;


//         //     SmartDashboard.putString("AAA_WinnerOfAuto", autowinner);

//         //     currentTime = LocalTime.now(); // get the time when the match starts

//         //     if (autowinner.equals(davecolour)) { // if we win auto

//         //         // if (currentTime.isBefore(autotimeshift)) {
//         //         // return true;
//         //         // }

//         //         if (currentTime.isAfter(auto_end)
//         //                 && currentTime.isBefore(first_shift_start)) { // if we're in the first period
//         //             return true;
//         //         } else if (currentTime.isAfter(first_shift_start) && currentTime.isBefore(second_shift_start)) {
//         //             return false;
//         //         } else if (currentTime.isAfter(second_shift_start)
//         //                 && currentTime.isBefore(third_shift_start)) { // if
//         //                                                        // we're
//         //                                                        // in
//         //                                                        // the
//         //                                                        // second
//         //                                                        // period
//         //             return true;
//         //         } else if (currentTime.isAfter(third_shift_start)
//         //                 && currentTime.isBefore(fourth_shift_start)) { // if
//         //                                                         // we're
//         //                                                         // in
//         //                                                         // the
//         //                                                         // third
//         //                                                         // period
//         //             return false;
//         //         } else { // the end of the match
//         //             return true;
//         //         }

//         //     } else { // if we lost auto
//         //         // if (currentTime.isBefore(autotimeshift)) {
//         //         // return true;
//         //         // }

//         //         if (currentTime.isAfter(auto_end)
//         //                 && currentTime.isBefore(first_shift_start)) {

//         //             return true;
//         //         } else if (currentTime.isAfter(first_shift_start) && currentTime.isBefore(second_shift_start)) {
//         //             return true;
//         //         } else if (currentTime.isAfter(second_shift_start)
//         //                 && currentTime.isBefore(third_shift_start)) {
//         //             return false;

//         //         } else if (currentTime.isAfter(third_shift_start)
//         //                 && currentTime.isBefore(fourth_shift_start)) {
//         //             return true;

//         //         } else {
//         //             return false;
//         //         }
//         //     }
//         // } else {
//         //     return true;
//         // }
//     }
// }
