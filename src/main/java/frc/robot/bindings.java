package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import java.lang.ModuleLayer.Controller;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;

import frc.robot.utils.swerve.swerve_lowlevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.autos;
import frc.robot.commands.compositions;
import frc.robot.subsystems.Owl_ware;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.neck;
import frc.robot.subsystems.outgive;
import frc.robot.subsystems.spindexer;
import frc.robot.subsystems.swerve;
import frc.robot.utils.auto_selector;
import frc.robot.utils.commandable_flag;
import frc.robot.utils.dave_led;
import frc.robot.utils.match_timer;
import frc.robot.utils.math_utils;
import frc.robot.utils.oi;
import frc.robot.utils.controls.dave_talon;
import frc.robot.utils.match_timer.phase;
import frc.robot.utils.swerve.swerve_request;

public final class bindings {

    final static int default_imu_mode = 2; // 2 is internal only
    public static int xkeys_LL_imu_mode = default_imu_mode;

    public static commandable_flag xkeys_always_shoot = new commandable_flag();

    public static boolean manual_intake = false;
    public static boolean should_backup_coral = false;
    public static boolean post_algae_flag = false;

    public static Trigger ctrl_reset_ll_heading = new Trigger(() -> false);

    public static void configure_bindings(robot robot) {

        // SWERVE STICKS
        final oi.shaping_chooser strafe_input_shaping = new oi.shaping_chooser("strafe_input_shaping");
        final oi.shaping_chooser turn_input_shaping = new oi.shaping_chooser("turn_input_shaping");
        final Supplier<Translation2d> ctrl_strafe = () -> oi.vec_deadband(oi.get_left_stick(oi.driver),
                strafe_input_shaping::shape);
        final Supplier<Double> ctrl_turn = () -> oi.deadband_precise(-oi.driver.getRightX(), turn_input_shaping::shape);

        // XKEYS
        ctrl_reset_ll_heading = oi.cmd_xkeys.button(11);

        var ctrl_trap = oi.cmd_xkeys.button(19);
        var ctrl_rp_down = oi.cmd_xkeys.button(20);
        var ctrl_email = oi.cmd_xkeys.button(21);
        var ctrl_folder = oi.cmd_xkeys.button(22);
        var ctrl_login = oi.cmd_xkeys.button(23);

        var ctrl_clear = oi.cmd_xkeys.button(16);

        // DRIVER BUTTONS
        var ctrl_reset_heading = oi.cmd_driver.povDown();
        var ctrl_dont_shoot = oi.cmd_driver.back().or(() -> !oi.cmd_driver.isConnected()); // left middle
        var ctrl_vomit = oi.cmd_driver.start(); // right middle
        var ctrl_deploy_intake = (oi.cmd_driver.rightTrigger().or( ctrl_trap )).and(DriverStation::isTeleopEnabled);
        var ctrl_manual = oi.cmd_driver.rightBumper();
        var ctrl_drive_slow = oi.cmd_driver.leftBumper();

        var ctrl_turret_stop_motor = oi.cmd_driver.x();
        var ctrl_start_climb = oi.cmd_driver.y();
        var ctrl_climb_auto_align = oi.cmd_driver.leftTrigger();



        ctrl_drive_slow.whileTrue( robot.swerve.manual_throttle.run() );

        ctrl_reset_ll_heading.onTrue(Commands.sequence(
                Commands.runOnce(() -> {
                    xkeys_LL_imu_mode = 1;
                }),
                Commands.waitSeconds(2),
                Commands.runOnce(() -> {
                    xkeys_LL_imu_mode = default_imu_mode;
                })));

        
        ctrl_rp_down.onTrue( Owl_ware.instance.goto_abs_angle(Degrees.of(0)) );

        ctrl_clear.toggleOnTrue(xkeys_always_shoot.run(true));

        // ctrl_aux3.onTrue( robot.owl_ware.zero() );
        // ctrl_aux8.onTrue(outgive.instance.auto_zero());
        var swerve = robot.swerve;
        ctrl_climb_auto_align.whileTrue(
                swerve.pid_to_point(constants::tower_align, 0.5, 0.0, 0.0, 1.0)
                        .alongWith(swerve.snap_auto_flip(180)));

        new Trigger(() -> !outgive.instance.isZerod() && DriverStation.isEnabled())
                .onTrue(
                        outgive.instance.auto_zero());

        new Trigger(() -> !intake.zeroed && DriverStation.isEnabled())
                .onTrue(
                        intake.instance.auto_zero());

        ctrl_start_climb.toggleOnTrue(compositions.auto_l1_climb(swerve));

        ctrl_reset_heading.onTrue(Commands.runOnce(() -> {
            swerve.zero_heading(frc.robot.robot.is_red() ? Rotation2d.k180deg : Rotation2d.kZero);
        }).ignoringDisable(true));

        var owl_idle = robot.owl_ware.idle_cmd();

        oi.cmd_driver.a().onTrue(robot.owl_ware.goto_abs_angle(constants.turret.owl_max_angle));
        oi.cmd_driver.b().onTrue(robot.owl_ware.goto_abs_angle(constants.turret.owl_min_angle));
        ctrl_turret_stop_motor.and(() -> !Owl_ware.instance.in_climb_state()).toggleOnTrue(
                owl_idle
        );

        ctrl_turret_stop_motor.and(Owl_ware.instance::in_climb_state).onTrue(
                Owl_ware.instance.end_climb());

        ctrl_trap.whileTrue(
                swerve.auto_intake()
        );

        ctrl_manual.whileTrue(Commands.parallel(
                outgive.instance.shoot_test()
        ));

        ctrl_manual.and(ctrl_deploy_intake.negate()).whileTrue(
                intake.instance.succ());

        new Trigger(DriverStation::isTeleopEnabled).onTrue(
                Commands.either(
                        compositions.declimb(swerve),
                        Commands.none(),
                        () -> Owl_ware.instance.in_climb_state()));

        Trigger auto_shoot = new Trigger(() -> activehubcheck(swerve)) // get rid of condition
                .and(robot::isTeleopEnabled)
                .and(ctrl_dont_shoot.negate())
                .and(() -> !Owl_ware.instance.in_climb_state())
                .and(ctrl_manual.negate())
                .and( () -> Owl_ware.instance.getDefaultCommand().isScheduled() );

        Trigger sotm_throttle = auto_shoot.and(() -> match_timer.is_hub_active()
                && game_data.my_field().alliance_zone().in_zone(swerve.get_turret_pose()));

        sotm_throttle.whileTrue(
                swerve.sotm_throttle.run());
        sotm_throttle.whileTrue(
                leds.flag_sotm_throttle.run());

        auto_shoot.whileTrue(outgive.instance.auto_shoot());

        // auto_shoot.and( ctrl_deploy_intake.negate() ).whileTrue(
        // spindexer.instance.SpinSlow()
        // );

        (auto_shoot.or( ctrl_manual )).and(ctrl_deploy_intake.negate()).and(() -> intake.hopper_deployed).whileTrue(
                intake.instance.jostle());

        final BooleanSupplier speedAccRequirements = () -> {
                if(!Owl_ware.instance.targeting_hub) {
                        return true;
                }
                var speed = swerve.get_speeds();
                boolean slow = Math.abs( speed.omegaRadiansPerSecond ) < 3.67 && math_utils.hypot(speed) < 1.9;
            try {
                double[] accelerationEstimate = swerve.acceleration_estimate();
                // accelerationEstimate.length is never zero
                final var criteria = math_utils.hypot(Math.abs(accelerationEstimate[0]),
                        Math.abs(accelerationEstimate[1])) < constants.swerve.max_acceleration_limit;
                return slow && criteria;
            } catch (Exception ignored) {
                return slow;
            }
        };

        new Trigger( outgive.instance::am_i_dead )
                .onTrue( outgive.instance.auto_zero() );

        var ready_to_shoot = outgive.instance.ready()
                .and(() -> Owl_ware.instance.at_target() || ctrl_manual.getAsBoolean())
                .and(speedAccRequirements)
                .and(swerve::is_flat)
                .and(() -> bindings.swerve_pose_shoot_ok(swerve));

        Trigger neck_feeding = (auto_shoot.or(autos.auton_shoot::get).or(ctrl_manual))
                .and(ready_to_shoot);

        neck_feeding.whileTrue(Commands.deadline(
                neck.instance.SpinNeck(),
                spindexer.instance.Spin(),
                leds.flag_nevk_feeding.run()));

        // hold right trigger to succ, intake stays down
        ctrl_deploy_intake.whileTrue(intake.instance.succ());
        ctrl_deploy_intake.whileTrue(swerve.intake_throttle.run());
        ctrl_deploy_intake.and(neck_feeding.negate()).whileTrue(
                spindexer.instance.SpinSlow());

        ctrl_vomit.whileTrue(intake.instance.vomit());

        // tap right bumper retracts intake
        // ctrl_retract_intake.and( ctrl_deploy_intake.negate() ).onTrue(
        // intake.instance.set_deploy_state( false )
        // );

        // ctrl_retract_intake.whileTrue(
        // intake.instance.jostle()
        // );

        swerve.strafe_subsystem.setDefaultCommand(swerve.tele_swerve_strafe(ctrl_strafe));
        swerve.omega_subsystem.setDefaultCommand(swerve.tele_swerve_omega(ctrl_turn));
    }

    private static boolean swerve_pose_shoot_ok(swerve mswerve) {
        Translation2d pose = mswerve.get_turret_pose();

        if (constants.never_zone.in_zone(pose)) {
            return false;
        }

        boolean distance_ok = true;

        double dist = Owl_ware.instance.target.getDistance(mswerve.get_turret_pose());

        if (dist > Units.inchesToMeters(217)) {
            distance_ok = false;
        }

        if (dist > 4.7 && math_utils.hypot(mswerve.get_speeds()) > 1.6) {
            distance_ok = false;
        }

        if (Owl_ware.instance.targeting_hub && !distance_ok) {
            return false;
        }

        return true;
    }

    private static boolean activehubcheck(swerve mswerve) {
        Translation2d pose = mswerve.get_turret_pose();
        SignalLogger.writeBoolean( "is hub active", match_timer.is_hub_active());
        SignalLogger.writeBoolean( "hub active preview", match_timer.hub_active_preview() );
        SignalLogger.writeBoolean( "hub active preview lookahead", match_timer.hub_active_preview(1.5) );
        SignalLogger.writeBoolean( "are we in aliance zone", game_data.my_field().alliance_zone().in_zone(pose) );
        SignalLogger.writeString( "current phase", match_timer.get_current_phase().name());
        if (xkeys_always_shoot.get()) {
            SignalLogger.writeBoolean( "check 1", true );
            return true;
        
        }

        SignalLogger.writeBoolean( "thing1", true );

        // fill up before on shift
        if ( !match_timer.is_hub_active() && match_timer.hub_active_preview() ) {
            if (match_timer.is_hub_active()) {
                SignalLogger.writeBoolean( "is hub active", true );
            }
            if (match_timer.hub_active_preview()) {
                SignalLogger.writeBoolean( "hub active preview", true );
            }
            SignalLogger.writeBoolean( "check 2", true );
            return false;
        }

        // dont score when not active and in zone
        if ( !(match_timer.hub_active_preview(1.5) || match_timer.is_hub_active() )
                && game_data.my_field().alliance_zone().in_zone(pose)) {
            SignalLogger.writeBoolean( "check 3", true );
            return false;
        }

        // dont pass when active and after transition
        if ( match_timer.is_hub_active() && !game_data.my_field().alliance_zone().in_zone(pose)
                && match_timer.get_current_phase() != phase.TRANSITION ) {
            SignalLogger.writeBoolean( "check 4", true );
            return false;
        }



        return true;
    }

}
