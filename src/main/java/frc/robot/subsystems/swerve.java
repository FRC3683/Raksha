package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.constants.swerve.*;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.LimelightHelpers;
import frc.robot.config;
import frc.robot.constants;
import frc.robot.robot;
import frc.robot.config.LL;
import frc.robot.utils.commandable_flag;
import frc.robot.utils.dummy;
import frc.robot.utils.field_util;
import frc.robot.utils.math_utils;
import frc.robot.utils.pololu_TOF;
import frc.robot.utils.controls.profiled_pid;
import frc.robot.utils.fsm_command.transition;
import frc.robot.utils.swerve.swerve_lowlevel;
import frc.robot.utils.swerve.swerve_request;
import frc.robot.utils.swerve.swerve_kin2.chassis_output;
import frc.robot.utils.controls.pid;

public class swerve extends swerve_lowlevel {

    private chassis_output cached_joystick_output = new chassis_output();
    
    private boolean reject_vision_updates = false;
    private Pose2d[] cam_pose = { new Pose2d(), new Pose2d() };
    private double[] last_vision_timestamps = { 0, 0 };

    public final pololu_TOF hood_climb_tof;

    public commandable_flag intake_throttle = new commandable_flag();
    public commandable_flag sotm_throttle = new commandable_flag();
    public commandable_flag manual_throttle = new commandable_flag();
    public commandable_flag snapshot = new commandable_flag();
    
    public final dummy strafe_subsystem, omega_subsystem;

    // IMPORTANT: dts is command dts, not swerve odom dts. Controller sets speeds in commands, wich run at default_period.
    protected final pid pid_x = new pid(config.swerve.strafe_to_point_config, constants.control_dts);
    protected final pid pid_y = new pid(config.swerve.strafe_to_point_config, constants.control_dts);
    protected final pid pid_hood = new pid(config.swerve.hood_align_config, constants.control_dts);
    protected final pid pid_strafe_velocity = new pid(config.swerve.strafe_velocity_config, constants.control_dts);
    protected final pid pid_heading_rad = new pid(config.swerve.heading_snap_config, constants.control_dts);
    protected final pid pid_omega_velocity = new pid(config.swerve.turn_velocity_config, constants.control_dts);
    protected final profiled_pid profiled_heading_rad = new profiled_pid(config.swerve.profiled_heading_snap_config, constants.control_dts);

    // 67
    public swerve(TimedRobot robot) {
        super(config.drive_canbus, config.swerve.module_configs, robot);
        strafe_subsystem = new dummy();
        omega_subsystem = new dummy();

        hood_climb_tof = new pololu_TOF( config.dio_hood_climb_tof, pololu_TOF.type.short_range_v2_50cm );

        SmartDashboard.putData( "swerve_snap_pid", pid_heading_rad );
        SmartDashboard.putData( "swerve_profiled_snap_pid", profiled_heading_rad );
        SmartDashboard.putData( "pid_to_point", pid_x );
        SmartDashboard.putData( "pid_hood_align", pid_hood );


        robot.addPeriodic(this::periodic, constants.control_dts);
    }

    private void periodic() {
        var heading = get_heading();
        var heading_rate = get_heading_rate();
        handle_vision_pose(config.LL_turret, 0, heading, heading_rate);
        // handle_vision_pose(config.LL_right, 1, heading, heading_rate);

        SmartDashboard.putBoolean( "swerve_flat", is_flat() );
        SmartDashboard.putBoolean( "swerve_slow_enough", math_utils.hypot( get_speeds() ) < 2.8 );

        var hood_climb_dist = hood_climb_tof.get_distance();
        hood_climb_dist.ifPresent( (Distance dist) -> {
            SmartDashboard.putNumber( "hood_climb_dist" , dist.in(Millimeters) );
        });
        SmartDashboard.putBoolean( "hood_climb_connected", hood_climb_tof.is_connected() );

        field.getObject( "balls" ).setPose(  new Pose2d( get_ll_autointake_coords(), Rotation2d.kZero ) );
    }

    public boolean is_flat() {
        return Math.abs( pig.getPitch().getValue().in(Degrees) ) < 10
            && Math.abs( pig.getRoll().getValue().in(Degrees) ) < 10;
    }

    public Command openloop_speeds( Supplier<Double> x_out, Supplier<Double> y_out, double torque_lim ) {
        swerve_request req = new swerve_request();
        return strafe_openloop(() -> {
            req.max_strafe_torque = torque_lim;
            req.target_field_relative.x_output = x_out.get();
            req.target_field_relative.y_output = y_out.get();
            return req;
        });
    }
    public Command openloop_speeds( double x_out, double y_out, double torque_lim ) {
        return openloop_speeds( () -> x_out, () -> y_out, torque_lim );
    }

    public Command openloop_speeds_auto_flip( double red_x_out, double red_y_out, double torque_lim ) {
        return openloop_speeds(() -> {
            return robot.is_red() ? red_x_out : -red_x_out;
        },
        () -> {
            return robot.is_red() ? red_y_out : -red_y_out;
        }, torque_lim) ;
    }

    private Translation2d get_ll_autointake_coords() {
        double[] oohaah = LimelightHelpers.getPythonScriptData(config.LL_intake.name);
        Translation2d pose = get_pose().getTranslation();

        if (snapshot.get()) {
            return new Translation2d(8.370, 3.997);
        }
        if (oohaah.length == 0) {
            return pose;
        }
    
        Translation2d target = pose.plus(new Translation2d(oohaah[0], oohaah[1]).rotateBy(get_heading()));

        field.getObject("ballz").setPose(target.getX(), target.getY(), Rotation2d.kZero);
        return target;
    }

    public Command auto_intake() {
        return pid_to_point( this::get_ll_autointake_coords, 0.6, 0, 0.1, 0.7 )
        .alongWith( snap_trans( this::get_ll_autointake_coords, 0 ) );
    }

    public Command hood_climb_lineup() {
        swerve_request req = new swerve_request();
        req.max_strafe_torque = 0.15;
        final double max_out = 0.25;
        final double target_m = 0.01;

        return Commands.runOnce( () -> {
            pid_hood.reset();
        } ).alongWith( strafe_openloop(() -> {
            var opt = hood_climb_tof.get_distance();

            if( !hood_climb_tof.is_connected() ) {
                req.target_field_relative.x_output = 0;
                req.target_field_relative.y_output = 0;
                return req;
            }

            // not close enough to the bar
            if( opt.isEmpty() ) {
                req.target_field_relative.x_output = robot.is_red() ? 0.3 : -0.3;
                req.target_field_relative.y_output = 0;
                return req;
            }

            double dist_m = opt.get().in( Meters );

            if( Math.abs(dist_m) < 0.01 ) {
                req.target_field_relative.x_output = 0;
                return req;
            }

            double output = pid_hood.calculate( target_m, dist_m );
            if( robot.is_red() ) {
                output = MathUtil.clamp( output, -0.01, max_out );
            } else {
                output = MathUtil.clamp( -output, -max_out, 0.01 );
            }
            req.target_field_relative.x_output = output;
            return req;
        }) ).until( () -> hood_climb_tof.get_distance().orElse( Meters.of( 0.4 ) ).in( Meters ) < target_m + 0.03 );
    }


    public Command into_depot( double x_output ) {
        swerve_request req = new swerve_request();
        Debouncer debounce = new Debouncer( 0.15 );
        return strafe_openloop(() -> {
            req.max_strafe_torque = 0.1;
            req.target_field_relative.x_output = x_output;
            return req;
        }).until( () -> debounce.calculate( get_field_relative_speeds().vxMetersPerSecond < 0.03 ) );
    }

    private void handle_vision_pose(LL ll, int index, Rotation2d heading, AngularVelocity heading_rate) {
        // LimelightHelpers.SetRobotOrientation(ll.name, heading.getDegrees(), 
        //     heading_rate.in(DegreesPerSecond), 0, 0, 0, 0); 67
        if( 
            // DriverStation.isAutonomousEnabled() || 
            RobotBase.isSimulation() ) {
            return;
        }
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll.name);

        if( DriverStation.isDisabled() ) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue( ll.name );
            if( mt1 == null ) {
                return;
            }
            boolean ok = true;
            ok = ok && mt1.tagCount >= 2;

            if( ok && Owl_ware.instance.crt_good() ) {
                Rotation2d theta = mt1.pose.getRotation().minus( Owl_ware.instance.get_crt().plus( Rotation2d.k180deg ) );
                // add_vision_measurement( , mt1.timestampSeconds, disabled_mt1_st_devs );
                reset_pose( new Pose2d( mt1.pose.getTranslation(), theta ) );
            }
            return;
        }

        if( mt2 == null ) {
            return;
        }

        if( mt2.pose.getX() == 0 && mt2.pose.getY() == 0) {
            // why
            return;
        }

        if(reject_vision_updates) {
            return;
        }


        cam_pose[index] = mt2.pose;
        field.getObject(ll.name).setPose(cam_pose[index]);
        ctre_log_pose(ll.name, cam_pose[index]);

        if(mt2.tagCount == 0) {
            return;
        }
        if(Math.abs(heading_rate.in(DegreesPerSecond)) > 360) {
            return;
        }
        if( mt2.avgTagDist > 6.7 || mt2.avgTagArea < 0.02 ) {
            return;
        }

        if(math_utils.close_enough(last_vision_timestamps[index], mt2.timestampSeconds, 0.02)) {
            // repeat vision measurement
        } else {
            add_vision_measurement(new Pose2d(cam_pose[index].getTranslation(), heading), mt2.timestampSeconds, mt2_st_devs);
            last_vision_timestamps[index] = mt2.timestampSeconds;
        }
    }

    private void reset_pose_now(LL ll) {
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll.name);
        if(mt2 != null && mt2.tagCount > 0 && mt2.pose != null) {
            reset_pose(new Pose2d(mt2.pose.getTranslation(), get_heading()));
        }
    }

    public Command reset_pose(LL ll) {
        return Commands.runOnce(() -> {
            reset_pose_now(ll);
        });
    }

    public boolean theta_within(double tol_deg) {
        return MathUtil.isNear(0, pid_heading_rad.getError(), Units.degreesToRadians(tol_deg));
    }

    public boolean theta_facing( double target, double tol_deg ) {
        double err = MathUtil.inputModulus( target - get_heading().getDegrees(), -180, 180 );
        return MathUtil.isNear( 0, err, tol_deg );
    }

    public Command cmd_reset_pose(Pose2d pose) {
        return Commands.runOnce(() -> {
            reset_pose(pose);
        }).ignoringDisable(true);
    }

    public Command cmd_reset_pose(Translation2d pose) {
        return Commands.runOnce(() -> {
            reset_pose(new Pose2d(pose, get_heading()));
        }).ignoringDisable(true);
    }

    public Command yeetSegment(Translation2d speed) {
        var req = new swerve_request();
        return strafe_openloop(() -> {
            Translation2d relativeSpeed;
        
            if (robot.is_red()) {
                relativeSpeed = speed.times( -1 ); // invert if on red
            } else {
                relativeSpeed = speed;
            }

            req.target_field_relative.x_output = relativeSpeed.getX();
            req.target_field_relative.y_output = relativeSpeed.getY();
            return req;
        });
    }

    public Command yeetOver(Translation2d speed) {
        return Commands.sequence(
            yeetSegment( speed ).until(() -> pig.getPitch().getValueAsDouble() > 10 ),
            yeetSegment( speed ).until(() -> pig.getPitch().getValueAsDouble() < 3 && pig.getPitch().getValueAsDouble() > -3)
        );
    }

    public void reject_vision(boolean do_reject) {
        reject_vision_updates = do_reject;
    }

    public Command cmd_reject_vision(boolean do_reject) {
        return Commands.runOnce(() -> {
            reject_vision(do_reject);
        });
    }

    public Command blind(Command to_wrap) {
        return to_wrap.alongWith(Commands.runOnce(() -> {
            reject_vision(true);
        })).finallyDo(() -> {
            reject_vision(false);
        });
    }

    public Command from_x() {
        swerve_request req = swerve_request.form_x();
        return Commands.runOnce(() -> {
            set_swerve_lowlevel(req);
        }).andThen(Commands.idle(strafe_subsystem, omega_subsystem));
    }

    public Command strafe_openloop(Supplier<swerve_request> periodic) {
        return Commands.run(() -> {
            set_swerve_lowlevel(periodic.get());
        }, strafe_subsystem);
    }

    public Command strafe_closedloop(Supplier<ChassisSpeeds> strafe_supplier) {
        var req = new swerve_request();
        return strafe_openloop(() -> {
            var target_speeds = math_utils.trans(strafe_supplier.get());
            var current_speeds = math_utils.trans(get_field_relative_speeds());
            var angle = (target_speeds.getNorm() == 0 ? current_speeds : target_speeds).getAngle();
            var target_speed = target_speeds.getNorm();

            var output = (target_speed / constants.swerve.max_speed_mps) + pid_strafe_velocity.calculate(current_speeds.getNorm(), target_speed);
            var out = new Translation2d(output, angle);

            req.target_field_relative.x_output = out.getX();
            req.target_field_relative.y_output = out.getY();
            return req;
        });
    }

    public Command strafe_omega_closedloop(Supplier<ChassisSpeeds> supplier) {
        return strafe_closedloop(supplier).alongWith(turn_closedloop(() -> supplier.get().omegaRadiansPerSecond));
    }

    public Command pid_to_point(final Supplier<Translation2d> point, final double max_output, final double tolerance, final double feedforward, final double torque_limit) {
        SmartDashboard.putNumber("swerve/pidpoint_err", 0);
        var req = new swerve_request().with_max_torque(torque_limit, 1.0);
        return Commands.runOnce(() -> {
            pid_x.setTolerance(tolerance);
            pid_x.reset();
            ctre_log_pose("strafe_to_point", point.get());
        })
        .alongWith(strafe_openloop(() -> {
            var err = point.get().minus(get_pose().getTranslation());
            double err_len = err.getNorm();
            SmartDashboard.putNumber("swerve/pidpoint_err", err_len);
            double speed = math_utils.clamp(-max_output, max_output, 
                pid_x.calculate(-err_len, 0) + feedforward
            );
            var output = err.div(err_len == 0 ? 1 : err_len).times(speed);
            req.target_field_relative.x_output = output.getX();
            req.target_field_relative.y_output = output.getY();
            return req;

        }).until(() -> point.get().getDistance(get_pose().getTranslation()) <= tolerance));
    }


    public Command pid_point_auto_flip(final Translation2d red_point, final double max_output, final double tolerance, final double feedforward, final double torque_limit, Translation2d red_offset, Translation2d blue_offset) {
        return pid_to_point( () -> {
            return robot.is_red() ? red_point.plus(red_offset) : field_util.flip( red_point ).plus(blue_offset);
        }, max_output, tolerance, feedforward, torque_limit );
    }

    // 67
    public Command pid_point_auto_flip(final Translation2d red_point, final double max_output, final double tolerance, final double feedforward, final double torque_limit) {
        return pid_point_auto_flip(red_point, max_output, tolerance, feedforward, torque_limit, new Translation2d(0, 0), new Translation2d(0, 0));
    }


    /*
     * TODO: I hate this.
     */
    public Command profile_to_point(final Supplier<Translation2d> point, final double max_speed, final double max_accel, final double tolerance, final double through_vel) {
        var constraints = new TrapezoidProfile.Constraints(max_speed, max_accel);
        var profile = new TrapezoidProfile(constraints);
        var goal_state = new TrapezoidProfile.State(0, through_vel);
        var current_state = new TrapezoidProfile.State(0, 0);
        final double kV = 0.2;
        final double kA = 0.1;
        var req = new swerve_request();
        return Commands.runOnce(() -> {
            pid_x.setTolerance(tolerance);
            pid_x.reset();
            ctre_log_pose("strafe_to_point", point.get());
        })
        .alongWith(strafe_openloop(() -> {
            var err = point.get().minus(get_pose().getTranslation());
            double err_len = err.getNorm();
            goal_state.position = err_len;
            current_state.velocity = get_field_relative_speeds().vxMetersPerSecond; // TODO
            var next_state = profile.calculate(constants.control_dts, current_state, goal_state);
            var accel = (next_state.velocity - current_state.velocity) / constants.control_dts;
            var feedforward = current_state.velocity * kV + accel * kA;
            var feedback = pid_x.calculate(-err_len, 0);
            var output = err.div(err_len == 0 ? 1 : err_len).times(feedforward + feedback);
            req.target_field_relative.x_output = output.getX();
            req.target_field_relative.y_output = output.getY();
            return req;
        }).until(() -> point.get().getDistance(get_pose().getTranslation()) <= tolerance));
    }


    // replace pid line? do I care about the line or the end velocity direction?
    public Command pid_to_point(Translation2d point, chassis_output through_vel, double max_output, double tolerance, double torque_limit) {
        assert tolerance > 0.03 : "must have tolerance for pid_to_point with chassis_output through_vel";

        if(through_vel.magnitude_squared() < 0.001) {
            return pid_to_point(() -> point, max_output, tolerance, tolerance, torque_limit);
        }

        var req = new swerve_request().with_max_torque(torque_limit, 1.0);
        return Commands.runOnce(() -> {
            pid_x.setTolerance(tolerance);
            pid_x.reset();
            ctre_log_pose("strafe_point???", point);
        })
        .alongWith(strafe_openloop(() -> {
            var direction = through_vel.trans().getAngle();
            var setpoint = point.rotateBy(direction.unaryMinus());
            var measurement = get_pose().getTranslation().rotateBy(direction.unaryMinus());
            var diff = setpoint.minus(measurement);

            // maybe replace magic 0.5 with smth based on chassis speeds?
            final double k = diff.getY() / ( Math.abs(diff.getX()) * 0.5 );
            final double sin = Math.sin( MathUtil.clamp(k, -1, 1) * Math.PI / 2.0 );

            final var target_angle = Rotation2d.fromRadians( sin * Math.PI / 2.0 ).rotateBy(direction);

            var out = new Translation2d(1.0, target_angle);

            req.target_field_relative.x_output = out.getX();
            req.target_field_relative.y_output = out.getY();

            return req;
        }));
    }

    // vector field?
    public Command pid_line(Translation2d point, Rotation2d direction, double max_output, double tolerance, final double k_feedforward, double torque_limit) {
        var req = new swerve_request().with_max_torque(torque_limit, 1.0);
        return Commands.runOnce(() -> {
            pid_x.setTolerance(tolerance);
            pid_x.reset();
            ctre_log_pose("strafe_line", point);
        })
        .alongWith(strafe_openloop(() -> {
            var setpoint = point.rotateBy(direction.unaryMinus());
            var measurement = get_pose().getTranslation().rotateBy(direction.unaryMinus());
            var diff = setpoint.minus(measurement);
            var distance = Math.abs(diff.getX()) + Math.abs(diff.getY()); // just trust me, bro
            var feedback = pid_x.calculate(-distance, 0);

            final double k = 0.9;

            var output_direction = Rotation2d.kZero;
            if(diff.getX() < 0) {
                output_direction = diff.getAngle();
            }
            else if(diff.getY() > 1) {
                output_direction = Rotation2d.kCCW_90deg;
            }
            else if(diff.getY() < -1) {
                output_direction = Rotation2d.kCW_90deg;
            }
            else if(diff.getX() > k) {
                output_direction = new Translation2d(k, diff.getY()).getAngle();
            } else {
                output_direction = diff.getAngle();
            }

            double feedforward = 0;
            if(diff.getX() > 0) {
                feedforward = k_feedforward * MathUtil.inverseInterpolate(90, 0, Math.abs(output_direction.getDegrees()));
            }

            var output = new Translation2d(MathUtil.clamp(feedforward+feedback, -max_output, max_output), output_direction);

            req.target_field_relative.x_output = output.getX();
            req.target_field_relative.y_output = output.getY();
            return req;
        }));
    }

    // in theory theres more wheel snapping with this approach
    public Command legacy_pid_line(Translation2d point, Rotation2d direction, double max_output, double tolerance, final double through_vel, double accel) {
        final double deadzone = Units.inchesToMeters(0.5);
        var req = new swerve_request();
        return Commands.runOnce(() -> {
            pid_x.reset();
            pid_y.reset();
            ctre_log_pose("strafe_to_point", point, direction.getDegrees());
        })
        .andThen(
            strafe_openloop(() -> {
                var setpoint = point.rotateBy(direction.unaryMinus());
                var measurement = get_pose().getTranslation().rotateBy(direction.unaryMinus());
                var y_out = math_utils.clamp(-max_output, max_output, pid_y.calculate(measurement.getY(), setpoint.getY()));
                // if(Math.abs(pid_y.getError()) < deadzone) {
                //     y_out = 0;
                //     pid_y.reset();
                // }
                var x_lim = Math.sqrt(max_output * max_output - (y_out * y_out));
                var x_feedback = pid_x.calculate(measurement.getX(), setpoint.getX());
                var x_out = math_utils.clamp(-x_lim, x_lim, x_feedback + through_vel * Math.signum(x_feedback));
                if(Math.abs(pid_x.getError()) < deadzone && x_out < 0.02) {
                    // x_out = 0;
                    // pid_x.reset();
                }
                var output = new Translation2d(
                    x_out,
                    y_out
                ).rotateBy(direction);
                output = math_utils.clamp(output, max_output);
                req.target_field_relative.x_output = output.getX();
                req.target_field_relative.y_output = output.getY();
                return req;
            }).until(() -> math_utils.close_enough(point, get_pose().getTranslation(), tolerance))
        );
    }

    public Command strafe_arc(Translation2d point, Translation2d arc_center, boolean clockwise, double max_vel, double tolerance) {
        final var arc_radius = point.minus(arc_center).getNorm();
        final var target_angle = point.minus(arc_center).getAngle();
        var req = new swerve_request();
        return strafe_openloop(() -> {
            var pose = get_pose().getTranslation();
            var curr_dist = pose.minus(arc_center);
            var curr_angle = curr_dist.getAngle();
            var target_dist = new Translation2d(arc_radius, curr_angle);
            var angle_err = math_utils.err(target_angle, curr_angle, clockwise);
            var dist_err = target_dist.minus(curr_dist);
            var x_lim = math_utils.remap_clamp(0.2, 0.9, Math.abs(dist_err.getNorm()), max_vel, 0);
            var err_arc = angle_err.getRadians() * curr_dist.getNorm();
            var arc_speed = math_utils.clamp(-x_lim, x_lim, pid_x.calculate(0, -err_arc));
            var dist_speed = math_utils.clamp(-max_vel, max_vel, pid_y.calculate(0, -dist_err.getNorm()));
            var arc_output = curr_dist.div(curr_dist.getNorm()).rotateBy(Rotation2d.fromDegrees(clockwise ? -90 : 90)).times(arc_speed);
            var center_output = curr_dist.div(curr_dist.getNorm()).times(-dist_speed);
            var output = arc_output.plus(center_output);
            req.target_field_relative.x_output = output.getX();
            req.target_field_relative.y_output = output.getY();
            return req;
        }).until(() -> math_utils.close_enough(point, get_pose().getTranslation(), tolerance));
    }

    public Command turn_openloop(Supplier<Double> omega_output_supplier) {
        return Commands.run(() -> {
            set_omega_lowlevel(omega_output_supplier.get());
        }, omega_subsystem);
    }

    public Command turn_closedloop(Supplier<Double> omega_rad_supplier) {
        return Commands.runOnce(() -> {
            pid_omega_velocity.reset();
        }).alongWith(turn_openloop(() -> {
            double omega_radps = omega_rad_supplier.get();
            double current_radps = get_field_relative_speeds().omegaRadiansPerSecond;
            return omega_radps / get_max_chassis_radps() + pid_omega_velocity.calculate(current_radps, omega_radps);
        }));
    }

    public Command maintain_heading() {
        var wrapper = new Object() { Rotation2d angle; };
        return Commands.sequence(
            Commands.runOnce(() -> {
                wrapper.angle = get_heading();
            }),
            snap(() -> wrapper.angle.getDegrees())
        );
    }

    public Command snap(Supplier<Double> theta_deg) {
        return snap_with_omega(() -> new theta_omega(Units.degreesToRadians(theta_deg.get()), 0));
    }
    public Command snap(double theta_deg) {
        return snap(() -> theta_deg);
    }
    public Command snap_auto_flip(double red_theta_deg) {
        return snap(() -> robot.is_red() ? red_theta_deg : field_util.flip( Rotation2d.fromDegrees( red_theta_deg ) ).getDegrees());
    }

    public Command profiled_snap( Supplier<Double> theta_deg_sup ) {
        return Commands.runOnce(() -> {
            profiled_heading_rad.reset(get_heading().getRadians(), get_heading_rate().in(RadiansPerSecond));
        }).alongWith(turn_openloop(() -> {
            State goal = new State(Math.toRadians( theta_deg_sup.get() ), 0);
            var output = profiled_heading_rad.calculate(get_heading().getRadians(), goal.position, goal.velocity);
            var state = profiled_heading_rad.get_state();
            SmartDashboard.putNumber("snap_target_pos", state.position);
            SmartDashboard.putNumber("snap_target_vel", state.velocity);
            SmartDashboard.putNumber("snap_current_pos", get_heading().getRadians());
            SmartDashboard.putNumber("snap_current_vel", get_speeds().omegaRadiansPerSecond);
            return output;
        }));
    }
    public Command profiled_snap_auto_flip( double red_theta_deg ) {
        return profiled_snap( () -> robot.is_red() ? red_theta_deg : field_util.flip( Rotation2d.fromDegrees( red_theta_deg ) ).getDegrees() );
    }



    // TODO hopefully this is no longer necessary
    public Command snap_deadzone(Supplier<Double> theta_deg, double deadzone, double debounce) {
        Debouncer deb = new Debouncer(debounce);
        return turn_openloop(() -> {
            double setpoint = Units.degreesToRadians(theta_deg.get());
            double measurement = get_heading().getRadians();
            var pid_out = pid_heading_rad.calculate(measurement, setpoint);
            if(deb.calculate( MathUtil.isNear(setpoint, measurement, deadzone) )) {
                return 0.0;
            }
            return pid_out;
        });
    }

    class theta_omega {
        double theta_rad, omega_radps;
        theta_omega(double theta_deg, double omega_degps) {
            this.theta_rad = theta_deg;
            this.omega_radps = omega_degps;
        }
    }

    // TODO: with omega may not be necessary now that we actually have PID instead of just P.... maybe thats a lie, theres a lot of natural damping keeping kD low....
    public Command snap_with_omega(Supplier<theta_omega> supplier) {
        return turn_openloop(() -> {
            var thetas = supplier.get();
            double strafe_k = math_utils.hypot(get_speeds()) / max_speed_mps;
            double limit = Math.sqrt(1 - strafe_k); //math_utils.remap(0, 1, strafe_k, 0.5, 0.1); // helps limit the "turns around random corner" thing
            double pid_output = MathUtil.clamp(pid_heading_rad.calculate(get_heading().getRadians(), thetas.theta_rad), -limit, limit);
            double feedback = pid_output;
            return feedback + (thetas.omega_radps / get_max_chassis_radps());
        });
    }

    public Command snap_medium_auto_flip( double red_degrees ) {
        return turn_openloop(() -> {
            double red_rad = Math.toRadians( red_degrees );
            double theta = robot.is_red() ? red_rad : field_util.flip( red_rad );
            double strafe_k = math_utils.hypot(get_speeds()) / max_speed_mps;
            double limit = Math.sqrt(1 - strafe_k); //math_utils.remap(0, 1, strafe_k, 0.5, 0.1); // helps limit the "turns around random corner" thing
            double pid_output = MathUtil.clamp(pid_heading_rad.calculate(get_heading().getRadians(), theta ), -limit, limit);
            double feedback = MathUtil.clamp( pid_output, -0.2, 0.2 );
            return feedback;
        });
    }

    public Command snap_trans( Supplier<Translation2d> look_at, Supplier<Double> offset_deg_sup ) {
        theta_omega thetas = new theta_omega(0, 0);
        return snap_with_omega(() -> {
            double offset_deg = offset_deg_sup.get();
            var pose = get_pose().getTranslation();
            var target = look_at.get();
            var diff = target.minus(pose);
            double target_rad = diff.getAngle().getRadians() + Units.degreesToRadians(offset_deg);
            var speeds = math_utils.trans(get_field_relative_speeds()).times(constants.control_dts);
            var next_pose = pose.plus(speeds);
            var next_diff = target.minus(next_pose);
            var next_target = next_diff.getAngle().getRadians() + Units.degreesToRadians(offset_deg);
            var ang_vel = (next_target - target_rad) / constants.control_dts;
            var vel_kP = 0.0;//1.0;//1;
            thetas.theta_rad = target_rad;
            thetas.omega_radps = ang_vel * vel_kP;
            return thetas;
        });
    }
    public Command snap_trans(Supplier<Translation2d> look_at, double offset_deg) {
        return snap_trans( look_at, () -> offset_deg );
    }
    public Command snap_trans(Translation2d look_at, double offset_deg) {
        return snap_trans(() -> look_at, offset_deg);
    }
    public Command snap_trans_auto_flip( Translation2d red_look_at, double red_offset_deg ) {
        return snap_trans(() -> {
            return robot.is_red() ? red_look_at : field_util.flip( red_look_at );
        },
        () -> {
            return red_offset_deg;
            // robot.is_red() ? red_offset_deg : field_util.flip( Rotation2d.fromDegrees( red_offset_deg ).getDegrees() );
        } );
    }

    public Command tele_swerve_strafe(Supplier<Translation2d> ctrl_strafe) {
        final String pref_key_drive_speed = "drive_speed", pref_key_intake_assist = "intake_assist", pref_key_torque_limit = "torque_limit_tele";
        final double pref_default_drive_speed = 10;
        final boolean pref_default_intake_assist = true;

        Preferences.initBoolean(pref_key_intake_assist, pref_default_intake_assist);
        Preferences.initDouble(pref_key_drive_speed, pref_default_drive_speed);

        var req = new swerve_request();
        return strafe_openloop(() -> {
            double throttle = 1.0;
            if(intake_throttle.get()) {
                throttle *= 0.9;
            }

            double torque_limit = 0.6;
            if( sotm_throttle.get() ) {
                torque_limit = 0.7;
                throttle *= 0.75;
            }

            if( manual_throttle.get() ) {
                throttle *= 0.467;
            }

            double pref = Preferences.getDouble(pref_key_drive_speed, pref_default_drive_speed);
            double scale = math_utils.remap(1, 10, pref, 0.1, 1.0);
            var max = scale * throttle;
            var min = constants.swerve.strafe_deadzone;
            var output = ctrl_strafe.get();
            
            if(output.getX() != 0 || output.getY() != 0) {
                var angle = output.getAngle();
                var length = output.getNorm();
                var new_length = length * (max - min) + min;
                output = new Translation2d(new_length, angle);
            }

            int flip = robot.is_red() ? -1 : 1;

            cached_joystick_output.move_towards(output.getX() * flip, output.getY() * flip, constants.control_dts / tele_slew_strafe.in(Seconds));
            req.target_field_relative.x_output = cached_joystick_output.x_output;
            req.target_field_relative.y_output = cached_joystick_output.y_output;
            req.turn_bias = 0.6;

            req.max_strafe_torque = torque_limit * 0.67;
            req.max_slip_angle = Degrees.of( 15 );
            return req;
        });
    }

    public Command tele_swerve_omega(Supplier<Double> ctrl_turn) {
        final String pref_key_turn_speed = "turn_speed";
        final double pref_default_turn_speed = 6.7;
        Preferences.initDouble(pref_key_turn_speed, pref_default_turn_speed);

        return turn_openloop(() -> {
            double throttle = 1.0;
            if( intake_throttle.get() ) {
                throttle *= 0.7;
            }
            double omega_in = ctrl_turn.get();
            double pref = Preferences.getDouble(pref_key_turn_speed, pref_default_turn_speed);
            double scale = math_utils.remap(1, 10, pref, 0.3, 1.0);
            var min = constants.swerve.omega_deadzone;
            var max = scale * throttle;
            double omega = 0;
            if(omega_in != 0) {
                omega = Math.signum(omega_in) * (Math.abs(omega_in) * (max - min) + min);
            }
            cached_joystick_output.omega_output = math_utils.move_toward(cached_joystick_output.omega_output, omega, constants.control_dts / tele_slew_omega.in(Seconds));
            double output = cached_joystick_output.omega_output;
            // if(Math.abs(output) < 0.01) {
                //     final double kD = 0.3;
                //     return MathUtil.applyDeadband(-get_heading_rate().in(RadiansPerSecond) * kD, Units.degreesToRadians(6));
                // }
            SmartDashboard.putNumber("turn_output", output);
            return output;
        });
    }

    public Command stop_motors() {
        var req = swerve_request.static_brake();
        return Commands.runOnce(() -> {
            set_swerve_lowlevel(req);
        }).andThen(Commands.idle(strafe_subsystem, omega_subsystem));
    }
}
