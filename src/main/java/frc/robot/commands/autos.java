package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants;
import frc.robot.robot;
import frc.robot.subsystems.Owl_ware;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.outgive;
import frc.robot.subsystems.spindexer;
import frc.robot.subsystems.swerve;
import frc.robot.subsystems.Owl_ware.track_type;
import frc.robot.utils.commandable_flag;
import frc.robot.utils.field_util;
import frc.robot.utils.auto_selector.auto;
import frc.robot.utils.swerve.swerve_request;
import frc.robot.utils.swerve.swerve_kin2.chassis_output;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.utils.auto_utils.*;

public final class autos {

    private static Timer auton_timer = new Timer();
    public static commandable_flag auton_shoot = new commandable_flag();

    private static final double quick_sweep_speed = 0.5;
    private static final double slow_sweep_speed = 0.32;


    // private static Command depot_side( swerve swerve, boolean climb ) {

    //     Command climb_ending = Commands.sequence(
    //         // towards tower
    //         async( swerve.profiled_snap_auto_flip( -90 ), auton_shoot.inc() ),
    //         swerve.pid_point_auto_flip( new Translation2d( 14.256, 3.32 ), 0.35, 0.2, 0.35, 1.0),
    //         async( swerve.profiled_snap_auto_flip( 180 ) ),
    //         swerve.pid_point_auto_flip( new Translation2d( 14.1, constants.tower_y_coord_red ), 0.35, 0.2, 0.35, 1.0),

    //         // under tower
    //         swerve.pid_point_auto_flip( new Translation2d( 15.0, constants.tower_y_coord_red ), 0.25, 0.03, 0.0, 1.0),

    //         Commands.either(
    //             Commands.sequence(
    //                 Commands.waitUntil( () -> auton_timer.get() > 17.5 ),
    //                 compositions.auto_l1_climb( swerve )
    //             ),
    //             Commands.none(),
    //             () -> auton_timer.get() < 17.4
    //         )   
    //     );

    //     Command fall_ending = Commands.sequence(
    //         async( swerve.profiled_snap_auto_flip( -90 ), auton_shoot.inc() ),
    //         swerve.pid_point_auto_flip( new Translation2d( 14.4, 2.23 ), 0.35, 0.2, 0.35, 1.0 ),
    //         async( swerve.profiled_snap_auto_flip( 180 ) ),
    //         swerve.pid_point_auto_flip( new Translation2d( 13.6, 2.58 ), 0.35, 0.05, 0.0, 1.0 )
    //     );

    //     return Commands.sequence(
    //         !climb ? shoot8(swerve) : Commands.none(),

    //         // yeet
    //         async(
    //             Commands.runOnce(() -> {
    //                 auton_timer.restart();
    //                 SmartDashboard.putBoolean("crossed", false);
    //             }),
    //             swerve.snap_auto_flip( 180 ),
    //             Owl_ware.instance.track( track_type.TIGHT_TRACK )
    //         ).alongWith(
    //             swerve.pid_point_auto_flip( new Translation2d( 10.6, 2.15 ), 1.0, 0.25, 1.0, 1.0)
    //         ),
    //         // start succ
    //         async(
    //             intake.instance.succ(),
    //             spindexer.instance.SpinSlow()
    //         ).alongWith(
    //             // corner
    //             swerve.pid_point_auto_flip( new Translation2d( 9.3, 2.024 ), 1.0, 0.13, 0.25, 1.0)
    //         ),
    //         // smooth turn
    //         async( swerve.snap_trans_auto_flip( new Translation2d(9.4, 2.7), 70 ) )
    //         .alongWith(
    //             swerve.pid_point_auto_flip( new Translation2d( 8.5, 2.609 ), quick_sweep_speed, 0.15, 0.5, 1.0)
    //         ),
    //         swerve.pid_point_auto_flip( new Translation2d( 8.7, 3.4 ), quick_sweep_speed, 0.15, 0.5, 1.0),
    //         async( Owl_ware.instance.goto_abs_angle( Degrees.of( -115 ) ) )
    //         .alongWith(
    //             swerve.pid_point_auto_flip( new Translation2d( 9.27, 3.5 ), quick_sweep_speed, 0.15, 0.5, 1.0)
    //         ),
    //         swerve.pid_point_auto_flip( new Translation2d( 9.78, 3.2 ), quick_sweep_speed, 0.15, 0.5, 1.0),
    //         // towards bump
    //         async(
    //             Owl_ware.instance.track( track_type.TIGHT_TRACK ),
    //             outgive.instance.auto_shoot(),
    //             swerve.snap_auto_flip( -29 )
    //         )
    //         .alongWith(
    //             swerve.pid_point_auto_flip( new Translation2d( 10.7, 2.7 ), 1.0, 0.25, 1.0, 1.0)
    //         ),
    //         // yeet
    //         Commands.parallel(
    //             async( swerve.snap_auto_flip( 0 ) ),
    //             async_delayed( 0.25,
    //                 Owl_ware.instance.track( track_type.SHOOT ),
    //                 auton_shoot.inc(),
    //                 intake.instance.pre_depot_jostle()
    //             ),
    //             swerve.pid_point_auto_flip( new Translation2d( 12.929, 2.2 ), 0.8, 0.17, 0.5, 1.0)
    //         ),
    //         // towards depot slow
    //         async(
    //             swerve.profiled_snap_auto_flip( 35 )
    //         ),
    //         swerve.pid_point_auto_flip( new Translation2d( 14.867, 1.1067 ), climb ? 0.25 : 0.15, 0.05, 0.17, 1.0),
            
    //         // depot
    //         depot( swerve ),
    //         async( intake.instance.jostle() ),
    //         swerve.pid_point_auto_flip( new Translation2d( 15.1, 1.9 ), 0.7, 0.07, 0.25, 1.0),

    //         ( climb ? climb_ending : fall_ending )
    //     );
    // }
    private static Command maybe_climb( swerve swerve ) {
        return maybe_climb( swerve, false );
    }

    private static Command maybe_climb( swerve swerve, boolean ignore_theta ) {
        return Commands.either(
            Commands.sequence(
                async(
                    swerve.snap_auto_flip( 180 ),
                    intake.instance.idle()
                ),
                Commands.waitUntil( () -> auton_timer.get() > 17 || ignore_theta ),
                compositions.auto_l1_climb( swerve, ignore_theta )
            ),
            Commands.none(),
            () -> auton_timer.get() < 16.5 || ignore_theta
        );
    }

    // private static Command weak( swerve swerve, boolean climb ) {

    //     Command climb_ending = Commands.sequence(
    //         // towards tower
    //         async( swerve.profiled_snap_auto_flip( -90 ) ),
    //         swerve.pid_point_auto_flip( new Translation2d( 14.256, 4.6 ), 0.35, 0.2, 0.35, 1.0),
    //         async( swerve.profiled_snap_auto_flip( 180 ) ),
    //         swerve.pid_to_point( new Translation2d( 14.1, constants.tower_y_coord_red ), 0.35, 0.2, 0.35, 1.0),

    //         // under tower
    //         swerve.pid_point_auto_flip( new Translation2d( 15.0, 4.27 ), 0.25, 0.02, 0.0, 1.0),

    //         Commands.either(
    //             Commands.sequence(
    //                 Commands.waitUntil( () -> auton_timer.get() > 17 ),
    //                 compositions.auto_l1_climb( swerve )
    //             ),
    //             Commands.none(),
    //             () -> auton_timer.get() < 16.5
    //         )
    //     );

    //     Command fall_ending = Commands.sequence(
    //         swerve.pid_point_auto_flip( new Translation2d( 14.256, 5.2 ), 0.35, 0.2, 0.35, 1.0)
    //     );

    //     return Commands.sequence(
    //         // shoot 8
    //         shoot8(swerve),

    //         // yeet
    //         async(
    //             auton_shoot.dec(),
    //             outgive.instance.idle()
    //         ).alongWith(
    //             swerve.pid_point_auto_flip( new Translation2d( 10.6, 6.0 ), 1.0, 0.25, 1.0, 1.0)
    //         ),
    //         // start succ
    //         async(
    //             intake.instance.succ(),
    //             spindexer.instance.SpinSlow()
    //         ).alongWith(
    //             // corner
    //             swerve.pid_point_auto_flip( new Translation2d( 9.3, 5.95 ), 1.0, 0.13, 0.4, 1.0)
    //         ),
    //         // smooth turn
    //         async( swerve.snap_trans_auto_flip( new Translation2d(9.4, 5.2), -70 ) )
    //         .alongWith(
    //             swerve.pid_point_auto_flip( new Translation2d( 8.5, 5.45 ), slow_sweep_speed, 0.15, 0.5, 1.0)
    //         ),
    //         swerve.pid_point_auto_flip( new Translation2d( 8.7, 4.63 ), slow_sweep_speed, 0.15, 0.5, 1.0),
    //         async( Owl_ware.instance.goto_abs_angle( Degrees.of( 115 ) ) )
    //         .alongWith(
    //             swerve.pid_point_auto_flip( new Translation2d( 9.27, 4.5 ), slow_sweep_speed, 0.15, 0.5, 1.0)
    //         ),
    //         swerve.pid_point_auto_flip( new Translation2d( 9.78, 4.8 ), slow_sweep_speed, 0.15, 0.5, 1.0),
    //         // towards bump
    //         async(
    //             Owl_ware.instance.track( track_type.TIGHT_TRACK ),
    //             outgive.instance.auto_shoot(),
    //             swerve.snap_auto_flip( 29 )
    //         )
    //         .alongWith(
    //             swerve.pid_point_auto_flip( new Translation2d( 10.7, 5.33 ), 1.0, 0.25, 1.0, 1.0)
    //         ),
    //         // yeet
    //         Commands.parallel(
    //             async( swerve.snap_auto_flip( 0 ) ),
    //             async_delayed( 0.15,
    //                 Owl_ware.instance.track( track_type.SHOOT ),
    //                 auton_shoot.inc(),
    //                 intake.instance.pre_depot_jostle()
    //             ),
    //             swerve.pid_point_auto_flip(  new Translation2d( 12.929, 5.3 ), 1.0, 0.17, 0.5, 1.0)
    //         ),

    //         ( climb ? climb_ending : fall_ending )
    //     );

        
    // }

    private static Command wiggle( swerve swerve, double red_deg ) {
        return Commands.repeatingSequence(
            swerve.snap_auto_flip( red_deg - 5.5 )
                .until( () -> swerve.theta_within( 2 ) ),
            swerve.snap_auto_flip( red_deg + 5.5 )
                .until( () -> swerve.theta_within( 2 ) )
        );
    }

    private static Command lowes( swerve swerve, boolean climb ) {

        Command climb_ending = Commands.sequence(
            async(
                swerve.profiled_snap_auto_flip(180)
            ).alongWith(
                swerve.pid_to_point( constants::tower_align, 0.15, 0.04, 0.05, 1.0 )
            ),
            async( wiggle( swerve, 180 ) )
            .alongWith(
                swerve.openloop_speeds(() -> 0.0, () -> 0.0, 1.0).until( () -> auton_timer.get() > 16 )
            ),
            maybe_climb( swerve )
        );

        Command no_climb_ending = Commands.sequence(
            async(
                wiggle( swerve, 180 )
            )
            .alongWith(
                swerve.pid_point_auto_flip( new Translation2d( 13.867, 5.9 ), 0.5, 0.0, 0.0, 1.0 )
                    .until( () -> auton_timer.get() > 17.5 )
            ),
            async(
                swerve.snap_auto_flip( 180 ),
                auton_shoot.dec(),
                intake.instance.succ()
            )
            .alongWith(
                swerve.pid_point_auto_flip( new Translation2d( 10, 5.9 ), 1.0, 0.0, 0.0, 1.0 )
            )
        );

        return Commands.sequence(
            async(
                Commands.runOnce(() -> {
                    auton_timer.restart();
                }),
                swerve.snap_auto_flip( -90 ),
                Owl_ware.instance.track( track_type.TIGHT_TRACK )
            )
            .alongWith(
                // yeet
                swerve.pid_point_auto_flip( new Translation2d( 10.6067, 5.98 ), 1.0, 0.25, 1.0, 1.0 )
            ),
            // out
            swerve.pid_point_auto_flip( new Translation2d( 10.1, 6.7 ), 1.0, 0.25, 1.0, 1.0 ),
            // prepare to succ
            async(
                swerve.snap_auto_flip( -75 ),
                intake.instance.succ(),
                spindexer.instance.SpinSlow()
            )
            .alongWith(
                // almost succ
                swerve.pid_point_auto_flip( new Translation2d( 8.9, 7.0 ), 1.0, 0.15, 0.6, 1.0 )
            ).withTimeout(3),
            // almost almost succ
            swerve.pid_point_auto_flip( new Translation2d( 8.28, 6.767 ), 1.0, 0.30, 0.6, 1.0 )
                .withTimeout(3),
            // succ
            swerve.pid_point_auto_flip( new Translation2d( 8.33, 5.60 ), 0.5, 0.1, 0.5, 1.0 )
                .withTimeout(3),
            swerve.pid_point_auto_flip( new Translation2d( 8.67, 4.50 ), 0.5, 0.1, 0.5, 1.0 )
                .withTimeout(3),
            // towards bump
            async( swerve.snap_auto_flip( 10 ) )
            .alongWith(
                swerve.pid_point_auto_flip( new Translation2d( 10.7,  5.26 ), 0.6, 0.1, 0.6, 1.0 )
            ),
            async_delayed( 0.67,
                outgive.instance.auto_shoot(),
                Owl_ware.instance.track( track_type.SHOOT ),
                auton_shoot.inc(),
                outgive.instance.set_extra_juice( true ),
                spindexer.instance.Spin(),
                intake.instance.pre_depot_jostle()
            )
            .alongWith(
                // yeet back
                swerve.pid_point_auto_flip( new Translation2d( 13.4, 5.65 ), 0.75, 0.25, 0.75, 1.0 )
            ),

            climb ? climb_ending : no_climb_ending
        );
    }

    private static Command home_depot( swerve swerve, boolean climb ) {

        Command climb_ending = Commands.sequence(
            async( swerve.profiled_snap_auto_flip( 180 ) )
            .alongWith(
                swerve.pid_point_auto_flip( new Translation2d( 14.8, 3.0 ), 0.25, 0.04, 0.25, 1.0 )
            ),
            swerve.pid_to_point( constants::tower_align, 0.25, 0.04, 0.05, 1.0 ),
            async( wiggle( swerve, 180 ) )
            .alongWith(
                swerve.openloop_speeds(() -> 0.0, () -> 0.0, 1.0).until( () -> auton_timer.get() > 16 )
            ),
            maybe_climb( swerve )
        );

        Command no_climb_ending = Commands.sequence(
            async(
                swerve.snap_auto_flip( -35 )
            ),
            // while turning
            swerve.openloop_speeds_auto_flip(0, 0.3, 1.0)
                .withTimeout( 0.2 ),
            Commands.waitUntil( () -> swerve.theta_within( 5 ) ),
            // in
            swerve.pid_point_auto_flip( new Translation2d( 15.80, 2.67 ), 0.3, 0.0967, 0.0, 1.0 )
                .withTimeout( 1.167 ),
            // away from wall
            swerve.openloop_speeds_auto_flip(-0.25, -0.25, 1.0)
                .withTimeout( 0.045 ),
            // back sweep
            swerve.openloop_speeds_auto_flip( 0, -0.3, 1.0 )
                .until( () -> robot.is_red() ? ( swerve.get_pose().getY() < 1.6 ) : ( swerve.get_pose().getY() > field_util.field_size_m.getY() - 1.6 ) ),
            
            swerve.pid_point_auto_flip( new Translation2d( 15.0, 1.1 ), 0.3, 0.0967, 0.0, 1.0 )
        );

        return Commands.sequence(
            async(
                Commands.runOnce(() -> {
                    auton_timer.restart();
                }),
                swerve.snap_auto_flip( 90 ),
                Owl_ware.instance.track( track_type.TIGHT_TRACK )
            )
            .alongWith(
                // yeet
                swerve.pid_point_auto_flip( new Translation2d( 10.6067, 2.0 ), 1.0, 0.25, 1.0, 1.0 )
            ),
            // out
            swerve.pid_point_auto_flip( new Translation2d( 10.1, 1.33 ), 1.0, 0.25, 1.0, 1.0 ),
            // prepare to succ
            async(
                swerve.snap_auto_flip( 75 ),
                intake.instance.succ(),
                spindexer.instance.SpinSlow(),
                Owl_ware.instance.track( track_type.PASS ),
                auton_shoot.inc()
            )
            .alongWith(
                // almost succ
                swerve.pid_point_auto_flip( new Translation2d( 8.9, 1.03 ), 1.0, 0.15, 0.6, 1.0 )
                    .withTimeout(3)
            ),
            // almost almost succ
            swerve.pid_point_auto_flip( new Translation2d( 8.28, 1.26 ), 1.0, 0.30, 0.6, 1.0 )
                .withTimeout(1.5),
                
            // succ
            swerve.pid_point_auto_flip( new Translation2d( 8.33, 2.45 ), 0.5, 0.1, 0.5, 1.0 )
                .withTimeout(2.75),
            swerve.pid_point_auto_flip( new Translation2d( 8.67, 3.44 ), 0.5, 0.1, 0.5, 1.0 )
                .withTimeout(3),
            // towards bump
            async( swerve.snap_auto_flip( 0 ) )
            .alongWith(
                swerve.pid_point_auto_flip( new Translation2d( 10.7,  2.77 ), 0.6, 0.1, 0.6, 1.0 )
            ),
            async_delayed( 0.67,
                outgive.instance.auto_shoot(),
                Owl_ware.instance.track( track_type.SHOOT ),
                // outgive.instance.set_extra_juice( true ),
                spindexer.instance.Spin(),
                intake.instance.pre_depot_jostle()
            )
            .alongWith(
                // yeet back
                swerve.pid_point_auto_flip( new Translation2d( 13.4, 2.38 ), 0.75, 0.25, 0.75, 1.0 )
            ),

            // slow towards depot
            async( swerve.profiled_snap_auto_flip( 35 ) )
            .alongWith(
                swerve.pid_point_auto_flip( new Translation2d( 15.0, 1.15 ), 0.15, 0.1, 0.75, 1.0 )
            ),

            async(
                intake.instance.succ(),
                spindexer.instance.SpinSlow(),
                swerve.snap_auto_flip( 35 )
            ),
            // first
            swerve.pid_point_auto_flip( new Translation2d( 15.4367, 1.3 ), 0.5, 0.05, 0.02, 1.0 ),
            // sweep
            swerve.pid_point_auto_flip( new Translation2d( 15.4367, 2.467 ), 0.25, 0.05, 0.2, 1.0 ),
            
            climb ? climb_ending : no_climb_ending
        );
    }


    private static Command niagara_lowes( swerve swerve, boolean climb ) {
        return Commands.sequence(
            async(
                Commands.runOnce(() -> {
                    auton_timer.restart();
                }),
                swerve.snap_auto_flip( 180 ),
                Owl_ware.instance.track( track_type.TIGHT_TRACK )
            )
            .alongWith(
                // yeet
                swerve.pid_point_auto_flip( new Translation2d( 10.6067, 5.067 ), 1.0, 0.25, 1.0, 1.0 )
            ).withTimeout(3),
            async(
                swerve.snap_auto_flip( 135 ),
                intake.instance.succ(),
                outgive.instance.auto_shoot()
            )
            // towards centerline
            .alongWith(
                swerve.pid_point_auto_flip( new Translation2d( 9.67, 3.967 ), 1.0, 0.367, 0.4, 1.0 )
            ).withTimeout(3),
            // balls
            swerve.pid_point_auto_flip( new Translation2d( 8.67, 4.067 ), 0.4, 0.15, 0.3, 1.0 ),
            async( swerve.snap_medium_auto_flip( 67 ) )
            .alongWith(
                async_delayed( 0.1, 
                    Owl_ware.instance.track( track_type.PASS ),
                    auton_shoot.inc()
                ),
                swerve.pid_point_auto_flip( new Translation2d( 8.291, 4.467 ), 0.367, 0.12, 1.0, 1.0 )
            ).withTimeout(3),
            async_delayed( 0.167,
                swerve.snap_medium_auto_flip( 41 )
            )
            .alongWith(
                swerve.pid_point_auto_flip( new Translation2d( 8.416, 6.7 ), 0.267, 0.45, 0.67, 1.0 )
            ).withTimeout(3),
            async(
                swerve.snap_auto_flip( -67 )
            )
            .alongWith(
                swerve.pid_point_auto_flip( new Translation2d( 8.416, 6.7 ), 0.267, 0.15, 0.1, 1.0 )
            ).withTimeout(3),
            swerve.pid_point_auto_flip( new Translation2d( 8.971, 6.4 ), 0.267, 0.15, 0.1, 1.0 ).withTimeout(3),
            swerve.pid_point_auto_flip( new Translation2d( 8.971, 5.1 ), 0.267, 0.25, 0.1, 1.0 ).withTimeout(3),
            // towards bump
            async(
                swerve.snap_auto_flip( 0 ),
                auton_shoot.dec(), // stop feeding
                Owl_ware.instance.track( track_type.TIGHT_TRACK )
            )
            .alongWith(
                swerve.pid_point_auto_flip( new Translation2d( 10.415, 5.476 ), 1.0, 0.25, 0.3, 1.0 )
            ).withTimeout(3),
            // yeet
            swerve.pid_point_auto_flip( new Translation2d( 13.5, 5.4967 ), 1.0, 0.45, 0.3, 1.0 ),
            // settle
            async(
                Owl_ware.instance.track( track_type.SHOOT ),
                auton_shoot.inc(),
                intake.instance.auton_jostle()
            )
            .alongWith(
                swerve.pid_point_auto_flip( new Translation2d( 14.6, 6.007 ), 0.267, 0.05, 0.0, 1.0 )
            ).withTimeout(3),
            async(
            intake.instance.vomit()
            ).alongWith(
                swerve.openloop_speeds_auto_flip( 0, 0, 1.0 )
                    .withTimeout( 1.0 )
            ).withTimeout(3),
            async(
                intake.instance.succ()
            ).alongWith(
                swerve.openloop_speeds_auto_flip( 0.1, 0, 1.0 )
                    .withTimeout( 1.0 )
            ).withTimeout(3),
            async(
                intake.instance.auton_jostle()
            ).alongWith(
                swerve.openloop_speeds_auto_flip( 0.0, 0, 1.0 )
                    .withTimeout( 1.0 )
            ).withTimeout(3)



            // async( swerve.snap_auto_flip( 0 ) )
            // .alongWith(
            //     swerve.pid_point_auto_flip( new Translation2d( 9.81, 7.39 ), 1.0, 0.15, 0.4, 1.0 )
            // ),
            // async( intake.instance.vomit() )
            // .alongWith(
            //     swerve.pid_point_auto_flip( new Translation2d( 11.11, 7.39 ), 1.0, 0.267, 0.0, 1.0 )
            // ),
            // // jostle vomit
            // swerve.pid_point_auto_flip( new Translation2d( 10.467, 7.39 ), 0.5, 0.167, 0.2, 1.0 ),
            // swerve.pid_point_auto_flip( new Translation2d( 11.11, 7.39 ), 0.5, 0.267, 0.1, 1.0 ),
            // swerve.pid_point_auto_flip( new Translation2d( 10.467, 7.39 ), 0.5, 0.167, 0.2, 1.0 ),
            // swerve.pid_point_auto_flip( new Translation2d( 11.11, 7.39 ), 0.5, 0.267, 0.1, 1.0 ),
            // // double dip
            // swerve.pid_point_auto_flip( new Translation2d( 10.267, 7.39 ), 1.0, 0.167, 0.6, 1.0 ),
            // async(
            //     intake.instance.succ(),
            //     swerve.snap_auto_flip( -120.0 )
            // )
            // .alongWith(
            //     swerve.pid_point_auto_flip( new Translation2d( 9.58, 7.09 ), 1.0, 0.067, 0.0, 1.0 )
            // )


        );
    }

    private static Command niagara_home_depot( swerve swerve, boolean climb ) {
        return Commands.sequence(
            async(
                Commands.runOnce(() -> {
                    auton_timer.restart();
                }),
                swerve.snap_auto_flip( 180 ),
                Owl_ware.instance.track( track_type.TIGHT_TRACK )
            )
            .alongWith(
                // yeet
                swerve.pid_point_auto_flip( new Translation2d( 10.433067, 8.036067 - 5.067 ), 1.0, 0.30067, 1.0, 1.0 )
            )
            .withTimeout(3),
            async(
                swerve.snap_auto_flip( -135 ),
                intake.instance.succ(),
                outgive.instance.auto_shoot()
            )
            // towards centerline
            .alongWith(
                swerve.pid_point_auto_flip( new Translation2d( 9.67, 4.069 ), 1.0, 0.367, 0.4, 1.0 )
            .withTimeout(3)),
            // balls
            swerve.pid_point_auto_flip( new Translation2d( 8.67, 3.969 ), 0.4, 0.15, 0.3, 1.0 ),
            async( swerve.snap_medium_auto_flip( -67 ) )
            .alongWith(
                async_delayed( 0.1, 
                    Owl_ware.instance.track( track_type.PASS ),
                    auton_shoot.inc()
                ),
                swerve.pid_point_auto_flip( new Translation2d( 8.291, 8.036 - 4.467 ), 0.367, 0.12, 1.0, 1.0 )
            ).withTimeout(3),
            async_delayed( 0.167,
                swerve.snap_medium_auto_flip( -41 )
            )
            .alongWith(
                swerve.pid_point_auto_flip( new Translation2d( 8.416, 8.036 - 6.7 ), 0.267, 0.45, 0.67, 1.0 )
            ).withTimeout(3),
            async(
                swerve.snap_auto_flip( 67 )
            )
            .alongWith(
                swerve.pid_point_auto_flip( new Translation2d( 8.416, 8.036 - 6.7 ), 0.267, 0.15, 0.1, 1.0 )
            ).withTimeout(3),
            swerve.pid_point_auto_flip( new Translation2d( 8.971, 8.036 - 6.4 ), 0.267, 0.15, 0.1, 1.0 ).withTimeout(3),
            swerve.pid_point_auto_flip( new Translation2d( 8.971, 8.036 - 5.1 ), 0.267, 0.25, 0.1, 1.0 ).withTimeout(3),
            // towards bump
            async(
                swerve.snap_auto_flip( 0 ),
                auton_shoot.dec(), // stop feeding
                Owl_ware.instance.track( track_type.TIGHT_TRACK )
            )
            .alongWith(
                swerve.pid_point_auto_flip( new Translation2d( 10.415, 8.036 - 5.476 ), 1.0, 0.25, 0.3, 1.0 )
            ).withTimeout(3),
            // yeet
            swerve.pid_point_auto_flip( new Translation2d( 13.5, 8.036 - 5.4967 ), 1.0, 0.45, 0.3, 1.0 ),
            // settle
            async(
                Owl_ware.instance.track( track_type.SHOOT ),
                auton_shoot.inc(),
                intake.instance.auton_jostle()
            )
            .alongWith(
                swerve.pid_point_auto_flip( new Translation2d( 14.367, 0.75 ), 0.267, 0.1, 0.0, 1.0 )
            ).withTimeout(3),
            async(
                wiggle( swerve, 0 )
            )
            .alongWith(
                swerve.openloop_speeds(0, 0, 0)
                .until(() -> auton_timer.get()>15)
            ),
            async(
                intake.instance.succ()

            ).
            alongWith(
                swerve.pid_point_auto_flip( new Translation2d( 16.011067, 0.449067 ), 0.1, 0.1, 0.0, 0.5 )
            )
            // async(
            //     intake.instance.vomit()
            // ).alongWith(
            //     swerve.openloop_speeds_auto_flip( 0, 0, 1.0 )
            //         .withTimeout( 1.0 )
            // ),
            // async(
            //     intake.instance.succ()
            // ).alongWith(
            //     swerve.openloop_speeds_auto_flip( 0.1, 0, 1.0 )
            //         .withTimeout( 1.0 )
            // ),
            // async(
            //     intake.instance.auton_jostle()
            // ).alongWith(
            //     swerve.openloop_speeds_auto_flip( 0.0, 0, 1.0 )
            //         .withTimeout( 1.0 )
            // )
        );
    }


    public static auto[] create( robot robot ) {

        var swerve = robot.swerve;
        SmartDashboard.putBoolean("crossed", false);


        return new auto[]{

            new auto( "HANG DEPOT",
                new Translation2d( 12.98, 2.15 ),
                home_depot( swerve, true )
            ),

            new auto( "LOWES",
                new Translation2d( 12.92, 5.98 ),
                lowes( swerve, true )
            ),

            new auto( "MIDDLE CUT DEPOT",
                new Translation2d( 12.98, 8.036 - 5.067 ),
                niagara_home_depot(swerve, false)
            ),

            new auto( "MIDDLE CUT OUTPOST",
                new Translation2d( 13.1, 5.067 ),
                niagara_lowes( swerve, false )
            ),

            new auto(),

            new auto( "HOME DEPOT",
                new Translation2d( 12.98, 2.15 ),
                home_depot( swerve, false )
            ),

            new auto( "LOWEST",
                new Translation2d( 12.92, 5.98 ),
                lowes( swerve, false )
            ),

            new auto("TEST", new Translation2d(), maybe_climb(swerve, true )),


            new auto("pid_line_3.0",
                null, 
                swerve.pid_to_point(new Translation2d(4.0, 1.0), new chassis_output(0.5, 0), 1.0, 0.1, 0.5)
            ),

            new auto("pid_line_0.5",
                new Translation2d(), 
                swerve.pid_line(new Translation2d(0.5, 0.0), Rotation2d.fromDegrees(0), 1.0, 0.0, 0, 0.5)
            ),

            new auto("pid_line_1.0",
                new Translation2d(), 
                swerve.pid_line(new Translation2d(1.0, 0.0), Rotation2d.fromDegrees(0), 1.0, 0.0, 0, 0.5)
            ),

        };

    }
    private static Command shoot8 (swerve swerve) {
        return async(
            Commands.runOnce(() -> {
                auton_timer.restart();
                SmartDashboard.putBoolean("crossed", false);
            }),
            swerve.snap_auto_flip( 180 ),
            Owl_ware.instance.track( track_type.SHOOT ),
            spindexer.instance.Spin(),
            outgive.instance.auto_shoot(),
            auton_shoot.inc()
        ).alongWith(
            Commands.waitSeconds( 1.4 )
        );
    }
}
