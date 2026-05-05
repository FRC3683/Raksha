package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.crt_encoder_pair;
import frc.robot.utils.controls.pid_config;
import frc.robot.utils.swerve.swerve_module;

public final class config {

    public static boolean is_comp = true;//!RobotController.getComments().equals("practice");
    static {
        SmartDashboard.putString("name", RobotController.getComments());
        SmartDashboard.putBoolean("is_comp", is_comp);
    }

    public static final boolean dev = true; // extra networktable values for code stuff

    public static final String main_canbus = "canivore";
    public static final String can_rio = "rio";
    public static final String drive_canbus = main_canbus;

    public static final int swerve_fl_turn = 1;
    public static final int swerve_fl_drive = 2;
    public static final int swerve_fl_cancoder = 11;

    public static final int swerve_fr_turn = 7;
    public static final int swerve_fr_drive = 8;
    public static final int swerve_fr_cancoder = 14;

    public static final int swerve_bl_turn = 3;
    public static final int swerve_bl_drive = 4;
    public static final int swerve_bl_cancoder = 12;

    public static final int swerve_br_turn = 5;
    public static final int swerve_br_drive = 6;
    public static final int swerve_br_cancoder = 13;

    public static final int can_pigeon = 10;

    public static final int bottomspindexer = 14;
    public static final int topspindexer = 15;

    public static final int neck_motor = 16;

    public static final int intake_arm_id = 17;
    public static final int intake_roller_id = 18;
    public static final int intake_roller_slave_id = 19;

    public static final int can_turret_crt_17 = 25;
    public static final int can_turret_crt_16 = 24;

    public static final int turret_owl_id = 20;
    public static final int turret_hood_id = 21;
    public static final int turret_flywheel_left_id = 22;
    public static final int turret_flywheel_right_id = 23;

    public static final int dio_hood_climb_tof = 0;

    public static final int pwm_leds = 0;


    public static final String LL_turret_name = "limelight-turret";
    public static final String LL_intake_name = "limelight-intake";
    public static final double pole_spacing_m = Units.inchesToMeters(13);
    public static final double ll_up_m = Units.inchesToMeters(6.75);
    public static final double ll_forward_m = Units.inchesToMeters(10);
    public static final Rotation2d ll_pitch = Rotation2d.fromDegrees(31);
    // TODO fix:
    public static final LL LL_turret = new LL(LL_turret_name, new Translation3d(ll_forward_m, pole_spacing_m/2.0, ll_up_m), ll_pitch);
    public static final LL LL_intake = new LL(LL_intake_name, new Translation3d(ll_forward_m, -pole_spacing_m/2.0, ll_up_m), ll_pitch);

    public final class neck {
        public static final TalonFXConfiguration roller_cfg = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted( InvertedValue.CounterClockwise_Positive )
            )
            .withSlot0(new Slot0Configs()
                .withKV( 0.124 )
                .withKP( 0.4 )
            )
            .withSlot1(new Slot1Configs()
                .withKV( 0.01 )
                .withKS( 0.01 )
                .withKP( 7 )
            )
            .withOpenLoopRamps(new OpenLoopRampsConfigs()
                .withDutyCycleOpenLoopRampPeriod( Milliseconds.of( 100 ) )
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit( 35 )
            )
        ;
    }

    public final class intake_cfg {
        public static final CurrentLimitsConfigs roller_auto_limits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit( 30 )
            .withStatorCurrentLimit( 65 )
        ;

        public static final CurrentLimitsConfigs roller_tele_limits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit( 30 )
            .withStatorCurrentLimit( 50 )
        ;

        public static final TalonFXConfiguration arm_cfg = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(200)
            )
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration( RotationsPerSecondPerSecond.of( 8 ) )
                .withMotionMagicCruiseVelocity( RotationsPerSecond.of( 1.5 ) )
                // CANIVORE BREAK DO THIS:
                // .withMotionMagicAcceleration( RotationsPerSecondPerSecond.of( 8 ) )
                // .withMotionMagicCruiseVelocity( RotationsPerSecond.of( 0.5 ) )
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit( 120 )
            )
            .withFeedback(new FeedbackConfigs()
                .withSensorToMechanismRatio( (30.0 / 12.0) * (64.0 / 16.0) * (50.0 / 20.0) * (36.0 / 12.0) )
            )
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted( InvertedValue.Clockwise_Positive )
            )
        ;

        public static final TalonFXConfiguration roller_cfg = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(0)
            )
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted( InvertedValue.CounterClockwise_Positive )
            )
            .withOpenLoopRamps( new OpenLoopRampsConfigs()
                .withVoltageOpenLoopRampPeriod( Milliseconds.of( 200 ) )
            )
            .withCurrentLimits( roller_auto_limits )
        ;
    }

    public final class turret_cfg {
        public static final TalonFXConfiguration owl_cfg = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs()
                .withSensorToMechanismRatio( (160.0 / 16.0) * (54.0 / 18.0) * (24.0 / 12.0) )
            )
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration( RotationsPerSecondPerSecond.of( 8 ) )
                .withMotionMagicCruiseVelocity( RotationsPerSecond.of( 5 ) )
            )
            .withSlot0(new Slot0Configs()
                .withKP( 167 )
                .withKA( 0.1067 )
                .withKV( 5.867 + 1.67 + 2.67 )
                .withKS( 0.39 )
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit( 85 )
                .withSupplyCurrentLimit( 40 )
            )
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted( InvertedValue.Clockwise_Positive )
            )
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true)
                .withForwardSoftLimitThreshold( constants.turret.owl_max_angle )
                .withReverseSoftLimitThreshold( constants.turret.owl_min_angle )
            )
        ;

        public static final TalonFXConfiguration hood_cfg = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs()
                .withSensorToMechanismRatio( ( 294.0 / 20.0 ) * ( 42.0 / 18.0 ) * ( 68.0 / 9.0 ) )
            )
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted( InvertedValue.CounterClockwise_Positive )
                .withNeutralMode(NeutralModeValue.Brake)
            )
            .withSlot0(new Slot0Configs()
                .withKP( 670 )
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit( 50 )
                .withSupplyCurrentLimit( 50 )
            )
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true)
                .withForwardSoftLimitThreshold( constants.turret.hardstop_hood_angle )
                .withReverseSoftLimitThreshold( constants.turret.flattest_hood_angle )
            )
        ;

        public static final TalonFXConfiguration left_flywheel_cfg = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs()
                .withSensorToMechanismRatio( 1.0 )
            )
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted( InvertedValue.CounterClockwise_Positive )
            )
            .withSlot0(new Slot0Configs()
                .withKP( 6767 )
            )
            .withTorqueCurrent(new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent( 120 )
                .withPeakReverseTorqueCurrent( -1 )
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit( 40 )
                .withStatorCurrentLimit( 120 )
            )
        ;

        public static final crt_encoder_pair.enc_cfg enc16_cfg = new crt_encoder_pair.enc_cfg(
            can_turret_crt_16, Rotations.of( -0.779052734375 ), 16
        );
        public static final crt_encoder_pair.enc_cfg enc17_cfg = new crt_encoder_pair.enc_cfg(
            can_turret_crt_17, Rotations.of( -0.708251953125 ), 17
        );
        public static final Angle crt_offset = Degrees.of( -306 );

        public static int big_teeth = 160;
    }

    public static class spindexer_cfg {
        public static final TalonFXConfiguration indexer_cfg = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs()
                .withSensorToMechanismRatio( (48.0 / 16.0) * (18.0 / 12.0) )
            )
            .withSlot0(new Slot0Configs()
                .withKP( 60 )
                .withKS( 5 )
                .withKV( 0.5 )
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit( 120 )
                .withSupplyCurrentLimit( 30 )
            )
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted( InvertedValue.Clockwise_Positive )
            )
            .withOpenLoopRamps(new OpenLoopRampsConfigs()
                .withDutyCycleOpenLoopRampPeriod( Milliseconds.of( 150 ) )
            )
            .withTorqueCurrent(new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(10)
            )
        ;
    }

    public final class swerve {

        public static final boolean ctre_pro = true;
        public static final boolean canivore = false;

        private static final double drive_kS = 0.22;
        private static final double max_speed_rotations_ps = constants.swerve.motor_free_speed.in(RotationsPerSecond) / constants.swerve.module.drive_ratio;// Units.radiansToRotations(constants.swerve.max_module_speed_mps / constants.swerve.wheel_radius);
        private static final double drive_kV = (12.0 - drive_kS) / max_speed_rotations_ps;
        public static final double coupling_kV = drive_kV * 0.5;
        public static final double skew_correction = 0.25;

        private static CurrentLimitsConfigs base_current_limits() {
            return new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit( Amps.of(50) )
                .withSupplyCurrentLimit( Amps.of(55) )
                .withSupplyCurrentLowerLimit( Amps.of(40) )
                .withSupplyCurrentLowerTime( Seconds.of(2) )
            ;
        }

        public static CurrentLimitsConfigs auto_limits = base_current_limits()
            .withStatorCurrentLimit( Amps.of( 125 ) )
            .withSupplyCurrentLimit( Amps.of( 80 ) )
        ;

        public static CurrentLimitsConfigs tele_limits = base_current_limits();


        public static TalonFXConfiguration drive_configs(InvertedValue inversion) {
            return new TalonFXConfiguration()
                .withCurrentLimits( tele_limits )
                .withOpenLoopRamps(new OpenLoopRampsConfigs()
                    .withVoltageOpenLoopRampPeriod( 0.01 )
                )
                .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio( constants.swerve.module.drive_ratio )
                )
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted( inversion )
                    .withNeutralMode( NeutralModeValue.Brake )
                    .withDutyCycleNeutralDeadband( 0.01 )
                )
                .withTorqueCurrent(new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent( 150 )
                    .withPeakReverseTorqueCurrent( -150 )
                )
                .withAudio(new AudioConfigs()
                    .withAllowMusicDurDisable( true )
                )
            ;
        }

        public static TalonFXConfiguration turn_configs( InvertedValue inversion, CANcoder cancoder ) {
            var closed_loop = new ClosedLoopGeneralConfigs();
            closed_loop.ContinuousWrap = true;
            return new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit( 80 )
                    .withStatorCurrentLimitEnable( true )
                    .withSupplyCurrentLimit( 30 )
                    .withSupplyCurrentLimitEnable( true )
                )
                .withFeedback(new FeedbackConfigs()
                    .withRotorToSensorRatio( constants.swerve.module.steer_ratio )
                    .withSensorToMechanismRatio( 1 )
                    .withFusedCANcoder( cancoder )
                )
                .withSlot0(new Slot0Configs()
                    .withKV( 12.0 / ( 5800.0 / 60.0 ) * constants.swerve.module.steer_ratio ) // (max volts / ( max speed ) ) * ratio
                    .withKP( 90.0 )
                    .withKD( 0.1 )
                )
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                    .withVoltageClosedLoopRampPeriod( 0.02 )
                )
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted( inversion )
                    .withNeutralMode( NeutralModeValue.Coast )
                )
                .withClosedLoopGeneral( closed_loop )
                .withAudio(new AudioConfigs()
                    .withAllowMusicDurDisable( true )
                )
            ;
        }

        public static final Pigeon2Configuration pig_config = new Pigeon2Configuration()
            .withMountPose(new MountPoseConfigs()
                .withMountPosePitch( Degrees.of(is_comp ? 0 : 0) )
                .withMountPoseYaw( Degrees.of( -90 ) )
            )
            .withGyroTrim(new GyroTrimConfigs()
                .withGyroScalarZ(-2)
            )
        ;

        public static pid_config strafe_to_point_config = new pid_config()
            .withKD( 0.0645 )
            .withGain( 1.45 )
            .with_integral_range( -1.0, 1.0 )
        ;

        public static pid_config hood_align_config = new pid_config()
            .withKD( 0.15 )
            .withGain( 1.1 )
            .with_integral_range( -1.0, 1.0 )
        ;

        public static pid_config strafe_velocity_config = new pid_config()
            .withGain( 0.1 )
        ;

        public static pid_config turn_velocity_config = new pid_config()
            .withGain( 0.1 )
        ;

        public static pid_config heading_snap_config = new pid_config()
            .withKP( 0.97 )
            .withKD( 0.067 )
            .with_continuous_input( -Math.PI, Math.PI )
        ;

        public static pid_config profiled_heading_snap_config = new pid_config()
            .withKP( 0.97 )
            .withKD( 0.067 )
            .with_continuous_input( -Math.PI, Math.PI )
            .with_trapezoid_constraints( 1.5, 2 )
            .with_feedforward( 0.08, 0.005 )
        ;

        public static final double pid_line_y_weight = 0.95;

        public static final swerve_module.module_config[] module_configs = {
            new swerve_module.module_config("fr", swerve_fr_drive, swerve_fr_turn, swerve_fr_cancoder,
                InvertedValue.Clockwise_Positive, InvertedValue.CounterClockwise_Positive,
                1.2372921578039222, 2.6765),

            new swerve_module.module_config("fl", swerve_fl_drive, swerve_fl_turn, swerve_fl_cancoder,
                InvertedValue.Clockwise_Positive, InvertedValue.CounterClockwise_Positive,
                0.8116644806852533, 3.8414),

            new swerve_module.module_config("br", swerve_br_drive, swerve_br_turn, swerve_br_cancoder,
                InvertedValue.Clockwise_Positive, InvertedValue.CounterClockwise_Positive,
                4.331455460343894, 0.6872),

            new swerve_module.module_config("bl", swerve_bl_drive, swerve_bl_turn, swerve_bl_cancoder,
                InvertedValue.Clockwise_Positive, InvertedValue.CounterClockwise_Positive,
                5.360418441746068, 2.7687),
        };
    }

    public final class leds {
        public static final int length = is_comp ? 20 : 20;
    }

    public static final class LL {
        public final String name;
        public final Translation3d mount_offset; // from center of robot, height from ground
        public final Rotation2d mount_angle; // from vertical to lense normal

        public LL(String name, Translation3d mount_offset, Rotation2d mount_angle) {
            this.name = name;
            this.mount_offset = mount_offset;
            this.mount_angle = mount_angle;
        }
    }

    public static class can {
        public int can_id;
        public String canbus;

        public can(int can_id, String canbus) {
            this.can_id = can_id;
            this.canbus = canbus;
        }
    }
}
