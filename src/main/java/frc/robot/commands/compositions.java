package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.constants;
import frc.robot.robot;
import frc.robot.subsystems.Owl_ware;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.outgive;
import frc.robot.subsystems.swerve;
import frc.robot.utils.swerve.swerve_request;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.utils.auto_utils.*;

public class compositions {


    public static Command auto_l1_climb( swerve swerve ) {
        return auto_l1_climb( swerve, false );
    }

    public static Command auto_l1_climb( swerve swerve, boolean ignore_theta ) {

        Command in = Commands.parallel(
            outgive.instance.hood( constants.turret.hardstop_hood_angle )
        );

        Command LL_align = Commands.deadline(
            Commands.waitUntil(() -> false), // TODO
            outgive.instance.hood( constants.turret.hardstop_hood_angle )
        );

        return Commands.sequence(
            async( swerve.snap( () -> robot.is_red() ? 180.0 : 0.0 ), intake.instance.vomit() ),

            // limelight align
            // Commands.waitSeconds( 0.05 ),
            Commands.waitUntil( () -> swerve.theta_within( 5 ) || ignore_theta ),

            // setup
            async( Owl_ware.instance.start_climb(), outgive.instance.hood( constants.turret.hardstop_hood_angle ) ),
            Commands.waitUntil( () -> 
                Owl_ware.instance.at_angle( 0, 3 )
                && outgive.instance.hood_within( constants.turret.hardstop_hood_angle, Degrees.of( 1.5 ) )
            ),

            // go in
            swerve.hood_climb_lineup(),

            async( outgive.instance.hood( constants.turret.flattest_hood_angle ) ),
            Commands.waitUntil( () -> outgive.instance.hood_within( constants.turret.flattest_hood_angle, Degrees.of( 1.5 ) ) ),

            // go out
            swerve.strafe_openloop( () -> {
                double x_out = robot.is_red() ? -0.5 : 0.5;
                return new swerve_request().with_chassis_output( x_out, 0, 0 );
            }).withTimeout( 0.15 ),
            async( outgive.instance.hood( constants.turret.hardstop_hood_angle ) )
        );
    }


    public static Command declimb( swerve swerve ) {
        return Commands.deadline(
            Commands.sequence(
                Commands.deadline(
                    Commands. sequence(
                        Commands.waitUntil( () -> outgive.instance.hood_within( constants.turret.flattest_hood_angle, Degrees.of( 1.5 ) ) ),
                        swerve.strafe_openloop( () -> {
                            double x_out = robot.is_red() ? 0.08 : -0.08;
                            return new swerve_request().with_chassis_output( x_out, 0, 0 ).with_max_torque( 0.15, 1 );
                        }).withTimeout( 0.29 )
                    ),
                    outgive.instance.hood( constants.turret.flattest_hood_angle )
                ),
                Commands.deadline(
                    Commands.sequence(
                        Commands.waitUntil( () -> outgive.instance.hood_within( constants.turret.hardstop_hood_angle, Degrees.of( 1.5 ) ) ),
                        swerve.strafe_openloop( () -> {
                            double x_out = robot.is_red() ? -0.5 : 0.5;
                            return new swerve_request().with_chassis_output( x_out, 0, 0 );
                        }).withTimeout( 0.7 )
                    ),
                    outgive.instance.hood( constants.turret.hardstop_hood_angle )
                ),
                async( end_current_command( swerve.omega_subsystem ) )
            ),
            swerve.snap( () -> robot.is_red() ? 180.0 : 0.0 )
        ).withInterruptBehavior( InterruptionBehavior.kCancelIncoming );
    }

}
