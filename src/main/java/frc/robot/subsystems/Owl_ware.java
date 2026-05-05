package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.Millimeters;

import java.util.ArrayList;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.config;
import frc.robot.constants;
import frc.robot.game_data;
import frc.robot.leds;
import frc.robot.robot;
// import frc.robot.utils.autoshootinfo;
import frc.robot.utils.configurable;
import frc.robot.utils.crt_encoder_pair;
import frc.robot.utils.hysteresis;
import frc.robot.utils.lazy_ctre;
import frc.robot.utils.lazy_localization;
import frc.robot.utils.math_utils;
import frc.robot.utils.pololu_TOF;
import frc.robot.utils.shotmap;
import frc.robot.utils.sotm;
import frc.robot.utils.shotmap.shot;

public class Owl_ware extends SubsystemBase implements configurable {
    public static Owl_ware instance;

    TalonFX m_turret = new TalonFX( config.turret_owl_id, config.main_canbus );

    private final crt_encoder_pair crt_pair = new crt_encoder_pair(
        config.turret_cfg.enc16_cfg,
        config.turret_cfg.enc17_cfg,
        config.main_canbus,
        config.turret_cfg.big_teeth,
        config.turret_cfg.crt_offset
    );

    public enum track_type {
        DYNAMIC,
        PASS,
        SHOOT,
        TIGHT_TRACK
    };

    private enum state {
        DEFAULT,
        CLIMB
    }
    private state current_state = state.DEFAULT;

    private final shotmap map, pass_map;
    private final InterpolatingDoubleTreeMap rpm_drop_predict;
    public static shotmap.shot tracking_shot = new shotmap.shot();

    private final Debouncer seen_tags_debouncer = new Debouncer( 0.67, DebounceType.kFalling );
    private boolean seen_tags = false;

    public boolean targeting_hub = false;
    public Translation2d target = new Translation2d();

    private track_type tracking_type = track_type.DYNAMIC;
    private double wrapped = 0, velocity = 0;

    private boolean stopped = false;

    public hysteresis at_angle_hyst = new hysteresis( 2.5, 5, false );

    swerve mswerve;
    private final double min_degrees = constants.turret.owl_min_angle.in( Degrees );
    private final double max_degrees = constants.turret.owl_max_angle.in( Degrees );

    private MotionMagicVoltage motionmagic_request = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
    private PositionVoltage tracking_request = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
    private boolean zerod = false;
    private AtomicBoolean configured = new AtomicBoolean( false );

    private final StatusSignal<Angle> position_signal;
    private final StatusSignal<AngularVelocity> velocity_signal;

    /** Creates a new Owl_ware. */
    public Owl_ware(swerve mswerve) {
        this.mswerve = mswerve;
        position_signal = m_turret.getPosition();
        velocity_signal = m_turret.getVelocity();

        frc.robot.robot.m_orchestra.addInstrument(m_turret);


        position_signal.setUpdateFrequency(100);
        velocity_signal.setUpdateFrequency(100);
        m_turret.optimizeBusUtilization();
        m_turret.stopMotor();
        map = new shotmap( 0, constants.shooer.shotmap );
        pass_map = new shotmap( -201.67, constants.shooer.pass_shotmap );

        rpm_drop_predict = new InterpolatingDoubleTreeMap();
        for( int i = 0; i < constants.shooer.rpm_drop_map.length; i += 2 ) {
            double rpm = constants.shooer.rpm_drop_map[i];
            double percent = constants.shooer.rpm_drop_map[i + 1];
            rpm_drop_predict.put( rpm, percent );
        }

        instance = this;

        SmartDashboard.putNumber("sotm fudge", 1.4);

        setDefaultCommand( track() );
    }

    public boolean crt_good() {
        return configured.get() && crt_pair.connected() 
            && crt_pair.get_value().lt( constants.turret.owl_max_angle )
            && crt_pair.get_value().gt( constants.turret.owl_min_angle );
    }

    public Rotation2d get_crt() {
        return new Rotation2d( crt_pair.get_value() );
    }

    public boolean at_angle( double target_deg, double tol_deg ) {
        return MathUtil.isNear( target_deg, position_signal.getValue().in(Degrees), tol_deg );
    }

    public boolean at_target() {
        if( !tracking_shot.valid || (!has_seen_tags() && targeting_hub) ) {
            return false;
        }
        return at_angle_hyst.get_value();
    }

    public boolean has_seen_tags() {
        return seen_tags;
    }

    public double get_degrees() {
        return position_signal.getValue().in( Degrees );
    }

    public Rotation2d get_rotation() {
        return new Rotation2d( position_signal.getValue() );
    }

    public Rotation2d get_field_relative_rotation() {
        return get_rotation().plus( mswerve.get_heading() );
    }

    public double get_degps() {
        return velocity_signal.getValue().in( DegreesPerSecond );
    }

    public Command zero() {
        return Commands.runOnce(() -> {
            m_turret.setPosition(0);
            zerod = true;
        }).ignoringDisable(true);
    }

    public Command idle_cmd() {
        return run(() -> {
            m_turret.setControl( new StaticBrake() );
            stopped = true;
        }).finallyDo(() -> {
            stopped = false;
        });
    }

    private void set_control( ControlRequest req ) {
        if( !zerod ) {
            m_turret.stopMotor();
            return;
        }
        if( current_state == state.CLIMB ) {
            m_turret.setControl( motionmagic_request.withPosition( 0 ) );
            return;
        }
        m_turret.setControl( req );
        // m_turret.stopMotor();
    }

    public Command goto_abs_angle( Angle angle ) {
        final double degrees = angle.in(Degrees);
        final double clamped_degrees = MathUtil.clamp( degrees, min_degrees, max_degrees );
        final Angle target = Degrees.of( clamped_degrees );
        return run(() -> {
            wrapped = target.in( Degrees );
            set_control( motionmagic_request.withPosition( target ) );
        });
    }

    public double wrapping( double target_angle ){

        target_angle = MathUtil.inputModulus( target_angle, -180, 180 );

        ArrayList<Double> candidates = new ArrayList<>();
        candidates.add(target_angle);
        candidates.add(target_angle - 360);
        candidates.add(target_angle + 360);

        double current_pos = m_turret.getPosition().getValue().in(Degrees);

        candidates.removeIf( (c) -> c < min_degrees || c > max_degrees );

        if( candidates.isEmpty() ) {
            return min_degrees;
        }

        double small = candidates.get(0);

        for(double c : candidates){
            //distance to target angle
            if (Math.abs(small - current_pos) > Math.abs(c - current_pos)){
                small = c;
            }
        };

        return small;
    }


    public void swerve_Turret(double current_pos){
        Pose2d swerve_pose = mswerve.get_pose();
        double swerve_x = swerve_pose.getTranslation().getX();
        double swerve_y = swerve_pose.getTranslation().getY();
        Rotation2d robot_heading = mswerve.get_heading();
    }

        
    private void set_position_turret( double turret_position ) {
        set_control( motionmagic_request.withPosition( turret_position ) );
    }

    // public void turret_angling(double turret_angle){
    //     double turret_num = wrapping(turret_angle);
    //     final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    //     set_control( m_request.withPosition(turret_num) );
    // };

    // public Command turret_move(){
    //     return run(()->{
    //         turret_angling(45);
    //     });
    // }

    double prev_tracked = 0;

    void tracking_logic() {
        double swerve_heading = mswerve.get_heading().getDegrees();
        double swerve_omega = Units.radiansToDegrees( mswerve.get_speeds().omegaRadiansPerSecond );

        double turret_deg = swerve_heading + get_degrees() + 180;

        Translation2d get = mswerve.get_turret_pose();
        var delta = target.minus( get );
        double target_degrees = delta.getAngle().getDegrees() - swerve_heading + 180;

        if( !leds.LL_connected_turret.connected() ) {
            target_degrees = -swerve_heading + ( robot.is_red() ? 0 : 180 );
        }

        prev_tracked = delta.getAngle().getDegrees();

        wrapped = wrapping( target_degrees );
        wrapped = MathUtil.clamp( wrapped , min_degrees, max_degrees );
        velocity = -swerve_omega * 1.50067;

    }

    public boolean in_climb_state() {
        return current_state == state.CLIMB;
    }

    public Command start_climb() {
        return run(() -> {
            current_state = state.CLIMB;
            set_control(null);
        });
    }

    public Command end_climb() {
        return runOnce(() -> {
            current_state = state.DEFAULT;
        });
    }

    public Command track() {
        return track( track_type.DYNAMIC );
    }

    public Command track( track_type type ) {
        return run(() -> {
            this.tracking_type = type;
            double dist_degrees = Math.abs( wrapped - get_degrees() );

            if( dist_degrees > 50 ) {
                set_control( motionmagic_request.withPosition( Degrees.of( wrapped ) ) );
            } else {
                set_control( tracking_request.withPosition( Degrees.of( wrapped ) )
                    .withVelocity( DegreesPerSecond.of( velocity ) )
                );
            }
        });
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        position_signal.refresh();
        velocity_signal.refresh();

        boolean tv = LimelightHelpers.getTV(config.LL_turret_name);
        seen_tags = seen_tags_debouncer.calculate( tv );
        double current_pos = m_turret.getPosition().getValue().in(Degrees);
        Pose2d swerve_pose = mswerve.get_pose();
        var swerve_degps = Units.radiansToDegrees( mswerve.get_field_relative_speeds().omegaRadiansPerSecond );

        double swerve_heading = mswerve.get_heading().getDegrees();
        double turret_deg = swerve_heading + get_degrees() + 180;
        Optional<Translation2d> me = lazy_localization.get_camera_pos();
        if( me.isPresent() ) {
            if( Math.abs( swerve_degps + get_degps() ) < 50 && tv && ( Math.abs( wrapped - get_degrees() ) < 15 || stopped || DriverStation.isDisabled() ) ) {
                mswerve.add_turret_vision_measurement( me.get() );
            }
        }

        SmartDashboard.putBoolean( "owl_at_target", at_target() );

        var turret_pos = mswerve.get_turret_pose();

        mswerve.field.getObject("turret").setPose( new Pose2d( turret_pos, Rotation2d.kZero ) );

        var delta = target.minus( turret_pos );
        double distance = delta.getNorm();
        SmartDashboard.putNumber("ll_dist", Units.metersToInches(distance) );

        if( ( game_data.my_field().alliance_zone().in_zone( turret_pos ) || tracking_type == track_type.SHOOT || tracking_type == track_type.TIGHT_TRACK ) && tracking_type != track_type.PASS ) {
            target = game_data.my_field().hub_position();
            targeting_hub = true;
            if( tracking_type == track_type.TIGHT_TRACK ) {
                tracking_shot = map.get_shot( 0 );
            } else {
                tracking_shot = map.get_shot( distance );
            }
        } else {
            targeting_hub = false;
            if( turret_pos.getY() < 4 ) {
                target = game_data.my_field().pass_low_y();
            } else {
                target = game_data.my_field().pass_high_y();
            }
            tracking_shot = pass_map.get_shot( distance );
        }

        SmartDashboard.putBoolean( "targeting_hub", targeting_hub );

        tracking_logic();

        if( tracking_type != track_type.TIGHT_TRACK ) {
            LinearVelocity exit_vel = MetersPerSecond.of( tracking_shot.vx * rpm_drop_predict.get( tracking_shot.vx ) );
            double fudge = SmartDashboard.getNumber("sotm fudge", 1.4);
            var adjusted = sotm.get_shot( mswerve.get_field_relative_speeds().times( fudge ), Pair.of( exit_vel, Rotation2d.fromDegrees(wrapped + swerve_heading + 180) ) );
            tracking_shot.vx = adjusted.getFirst().in(MetersPerSecond) / rpm_drop_predict.get( tracking_shot.vx );
            wrapped = adjusted.getSecond().getDegrees() - swerve_heading + 180;
        }

        wrapped = wrapping(wrapped);

        Rotation2d robot_heading = mswerve.get_heading();
        crt_pair.refresh();
        if( !zerod && crt_good() ) {
            m_turret.setPosition( crt_pair.get_value() );
            zerod = true;
        }

        SmartDashboard.putBoolean("crt_zerod", zerod);


        if( zerod
        //  && Math.abs( swerve_degps + get_degps() ) < 30
        //  && ( Math.abs( wrapped - get_degrees() ) < 20 || stopped )
        ) {
            // double k = math_utils.remap_clamp(0, 360, Math.abs( swerve_degps + get_degps() ), 1, 0.001);

            double ll_alpha = 0.0010067;
            // if( DriverStation.isDisabled() ) {
            // LimelightHelpers.SetIMUMode("limelight-turret", 1);

                // LimelightHelpers.setLimelightNTDouble( "limelight-turret", "imuassistalpha_set", ll_alpha );
            double crt = crt_pair.get_value().in(Degrees);
            LimelightHelpers.SetRobotOrientation(config.LL_turret_name, swerve_heading + crt + 180, 0, 0, 0, 0, 0);
            // }
        } else {
            // LimelightHelpers.SetIMUMode("limelight-turret", 2);

        }

        at_angle_hyst.refresh( get_degrees(), wrapped );

        SmartDashboard.putBoolean( "crt_enc16", crt_pair.is_enc1_connected() );
        SmartDashboard.putBoolean( "crt_enc17", crt_pair.is_enc2_connected() );
        SmartDashboard.putNumber( "crt_enc16_abs", crt_pair.enc1_angle().in(Degrees) );
        SmartDashboard.putNumber( "crt_enc17_abs", crt_pair.enc2_angle().in(Degrees) );
        if( crt_pair.connected() ) {
            SmartDashboard.putNumber( "crt_angle", crt_pair.get_value().in(Degrees) );
        }
    }

    @Override
    public void configure() {
        m_turret.getConfigurator().apply( config.turret_cfg.owl_cfg );
        configured.set( true );
        
    }

}
