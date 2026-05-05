// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.atomic.AtomicBoolean;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.commands.autos;
import frc.robot.commands.test_autos;
import frc.robot.simulation.mech2d;
import frc.robot.utils.auto_selector;
import frc.robot.utils.configurable;
import frc.robot.utils.ctre_logs;
import frc.robot.utils.dave_preferences;
import frc.robot.utils.field_util;
import frc.robot.utils.lazy_localization;
import frc.robot.utils.limelight;
import frc.robot.utils.match_timer;
import frc.robot.utils.uptime;
import frc.robot.utils.voltage_warning;
import frc.robot.subsystems.Owl_ware;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.neck;
import frc.robot.subsystems.outgive;
import frc.robot.subsystems.spindexer;
import frc.robot.subsystems.swerve;

public final class robot extends TimedRobot {

    public final swerve swerve;
    public final Owl_ware owl_ware;
    public final spindexer spinny_boi;
    public final intake ball_sucker;

    public final leds leds;
    private final auto_selector selector;
    public final PowerDistribution pd;
    private Command auto_command;
    public limelight m_limelight = new limelight("limelight-right");
    public Field2d m_Field;
    // public Optional<Translation2d> limelight_pose_estimate;
    public static Mechanism2d three_tag_case_mechanism = new Mechanism2d(16.54, 8);
    public static MechanismRoot2d zero_zero = three_tag_case_mechanism.getRoot("zero_zero", 0, 0);
    public static Orchestra m_orchestra = new Orchestra();

    public static MechanismLigament2d tag1display = zero_zero.append(new MechanismLigament2d("tag1display", 0, 0, 5, new Color8Bit("#00FFFF")));
    public static MechanismLigament2d tag2display = zero_zero.append(new MechanismLigament2d("tag2display", 0, 0, 5, new Color8Bit("#006840")));
    public static MechanismLigament2d tag3display = zero_zero.append(new MechanismLigament2d("tag3display", 0, 0, 5, new Color8Bit("#004949")));

    
    // pololu_TOF tof = new pololu_TOF(3, pololu_TOF.type.short_range_og_50cm);

    private final configurable[] configurables;
    public static boolean is_red() {
        var a = DriverStation.getAlliance();
        return a.isPresent() && a.get().equals(Alliance.Red);
    }

    public robot() {
        super(constants.control_dts);

        swerve = new swerve(this);
        ball_sucker = intake.instance;
        owl_ware = new Owl_ware(swerve);
        spinny_boi = spindexer.instance;

        configurables = new configurable[]{ swerve, ball_sucker, owl_ware, spinny_boi, neck.instance, outgive.instance };

        leds = new leds(this, swerve);
        pd = new PowerDistribution();
        selector = new auto_selector( autos.create(this), test_autos.create(this), swerve::reset_pose );

        var tags = constants.tags;
        tags.getTagPose(0); // force constants static initialization
        m_Field = new Field2d();

        // autoshootinfo.shouldshootperiodic(); // force static init
    }

    @Override
    public void robotInit() {
        dave_preferences.save_file();

        bindings.configure_bindings(this);
        debug.add_dashboard_commands(this);
        selector.init();
        voltage_warning.set_thresholds(constants.voltage_warning_threshold_comp, constants.voltage_warning_threshold_prac);
        voltage_warning.add_nt_listener(this, pd, 2);
        // can_savior.begin(this);
        uptime.init(this);
        configurable.configure_all(leds.flag_configuring.run(), configurables);

        // ctre_logs.init();

        // SignalLogger.enableAutoLogging(false);

        SmartDashboard.putData(" three tag case mechanism", three_tag_case_mechanism);
        SmartDashboard.putNumber("tag 1 length", 0);
        SmartDashboard.putNumber("tag 2 length", 0);
        SmartDashboard.putNumber("tag 3 length", 0);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber( "match_time", Math.ceil( match_timer.end_of_match - match_timer.match_seconds_elapsed() ) );
        SmartDashboard.putNumber("shift time", Math.ceil( match_timer.seconds_left_in_shift() ) );
        SmartDashboard.putBoolean( "active", match_timer.is_hub_active() );

        var LL_imu = LimelightHelpers.getIMUData(config.LL_turret_name);
        boolean has_imu = LL_imu.Yaw != 0;
        SmartDashboard.putBoolean("LL IMU ok", has_imu);

        // SmartDashboard.putNumber("tof_connected", tof.is_connected() ? 1 : 0);
        // SmartDashboard.putNumber("tof_get", tof.get() ? 1 : 0);
        // SmartDashboard.putNumber("tof_dist", tof.get_distance().orElse(Meters.of(-1)).in(Meters));
        
        // var pose = lazy_localization.get_turret_pos( owl_ware.get_field_relative_rotation() );
        // if( pose.isPresent() ) {
        //     Translation2d us = pose.get();
        //     double dist = constants.red_hub.getDistance( us );
        //     SmartDashboard.putNumber( "distance", dist );
        // }
    }

    private Timer disabledTimer = new Timer();

    @Override
    public void disabledInit() {
        intake.instance.apply_auto_limits();
        disabledTimer.restart();
    }

    @Override
    public void disabledPeriodic() {
        // throttle LL4 to avoid overheading while sitting disabled:
        LimelightHelpers.SetIMUMode("limelight-turret", 1);
        if( DriverStation.isTeleop() ) {
            intake.instance.apply_tele_limits();
            swerve.apply_current_limits( false );
        } else {
            intake.instance.apply_auto_limits();
            swerve.apply_current_limits( true );
        }
        // LimelightHelpers.setPipelineIndex(config.LL_turret_name, 1);
        boolean no_throttle = ( disabledTimer.get() < 5 || bindings.ctrl_reset_ll_heading.getAsBoolean() );
        LimelightHelpers.setLimelightNTDouble("limelight-turret", "throttle_set", no_throttle ? 0 : 200 );

        Translation2d please = selector.get_preferred_start_pose();
        if( please != null ) {
            boolean close = please.getDistance( swerve.get_pose().getTranslation() ) < 0.2;
            SmartDashboard.putBoolean( "close_enough", close );
        } else {
            SmartDashboard.putBoolean( "close_enough", false );
        }
    }

    @Override
    public void disabledExit() {
        SmartDashboard.putBoolean( "close_enough", true );
    }

    @Override
    public void autonomousInit() {
        auto_command = selector.get_auto_command();
        if(auto_command != null) {
            auto_command.schedule();
        }
        match_timer.auto_init();
        // autoshootinfo.autosetup();
    }

    @Override
    public void autonomousPeriodic() {
        // stop throttling LL4 to avoid overheading while sitting disabled:
        LimelightHelpers.setPipelineIndex(config.LL_turret_name, 0);
        LimelightHelpers.setLimelightNTDouble("limelight-turret", "throttle_set", 0);

        LimelightHelpers.SetIMUMode("limelight-turret", 2); // 2 is internal only
        // autoshootinfo.shouldshootperiodic();
    }

    @Override
    public void autonomousExit() {
        autos.auton_shoot.reset();
        intake.instance.apply_tele_limits();
        swerve.apply_current_limits( false );
    }

    @Override
    public void teleopInit() {
        // autoshootinfo.telesetup();
        match_timer.tele_init();
    }

    @Override
    public void teleopPeriodic() {
        // stop throttling LL4 to avoid overheading while sitting disabled:
        LimelightHelpers.setPipelineIndex(config.LL_turret_name, 0);
        LimelightHelpers.setLimelightNTDouble("limelight-turret", "throttle_set", 0);

        LimelightHelpers.SetIMUMode( "limelight-turret", bindings.xkeys_LL_imu_mode );
        // autoshootinfo.shouldshootperiodic();
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        swerve.stop_motors().withInterruptBehavior(InterruptionBehavior.kCancelIncoming).schedule();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
        CommandScheduler.getInstance().cancelAll();
    }
}