package frc.robot.utils.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.swerve.*;

import java.util.Optional;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.HootReplay;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.config;
import frc.robot.constants;
import frc.robot.utils.configurable;
import frc.robot.utils.lazy_localization;
import frc.robot.utils.math_utils;
import frc.robot.utils.swerve_mech2d;
import frc.robot.utils.swerve.swerve_kin2.chassis_output;
import frc.robot.utils.swerve.swerve_kin2.module_output;
import frc.robot.utils.swerve.swerve_kin2.module_speed;
import frc.robot.utils.swerve.swerve_module.module_config;

public abstract class swerve_lowlevel implements configurable {

    protected final Pigeon2 pig = new Pigeon2(config.can_pigeon, config.drive_canbus);
    public final swerve_module[] modules;

    private final StatusSignal<Angle> heading_signal;
    private final StatusSignal<AngularVelocity>  heading_rate_signal_odom, heading_rate_signal;
    private final StatusSignal<Angle>  pitch_signal;
    // CTRE recommends seperate status signals for seperate threads

    protected final swerve_kin2 kin = new swerve_kin2(
        module_offsets, new Translation2d(constants.swerve.turret_offset, 0)
    );
    private SwerveDrivePoseEstimator pose_estimator;
    private SwerveDrivePoseEstimator turret_pose_estimator;
    private final swerve_output_optimizer optimizer = new swerve_output_optimizer(
        MetersPerSecond.of(constants.swerve.max_module_speed_mps), Degrees.of(22), odom_dts ); // TODO should be smth smaller like 20 degree slip angle I think

    private final module_output[] form_x_states = kin.form_x();

    private final ReadWriteLock input_lock = new ReentrantReadWriteLock();
    private final swerve_request default_request = new swerve_request();
    private final swerve_request default_static_brake = swerve_request.static_brake();
    private swerve_request request = default_static_brake;

    private final ReadWriteLock state_lock = new ReentrantReadWriteLock();
    private Pose2d pose = new Pose2d();
    private final chassis_output commanded_field_relative_output = new chassis_output();
    private final ChassisSpeeds current_field_relative_speeds = new ChassisSpeeds();

    private final module_output[] desired_states = {
        new module_output(0, 0, 0), 
        new module_output(0, 0, 0), 
        new module_output(0, 0, 0), 
        new module_output(0, 0, 0),
    };
    private final module_speed[] current_states = {
        new module_speed(0, 0, 0), 
        new module_speed(0, 0, 0), 
        new module_speed(0, 0, 0), 
        new module_speed(0, 0, 0),
    };
    private final SwerveModulePosition[] module_positions = {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
    };

    public final Field2d field = new Field2d();
    private final akit_swerve_vis swerve_vis = new akit_swerve_vis();

    private final odometry_thread odom_thread;

    /**
     * heavily inspired by (stolen from) from CTRE LegacySwerveDrivetrain
     */
    /* Perform swerve module updates in a separate thread to minimize latency */
    private class odometry_thread {
        private static final int START_THREAD_PRIORITY = 1; // Testing shows 1 (minimum realtime) is sufficient for tighter
                                                            // odometry loops.
                                                            // If the odometry period is far away from the desired frequency,
                                                            // increasing this may help

        private final Thread thread;
        private volatile boolean is_running = false;

        private final BaseStatusSignal[] all_signals = {
            heading_signal, heading_rate_signal_odom,
            modules[0].drive_pos_signal, modules[0].drive_vel_signal, modules[0].turn_pos_signal, modules[0].turn_vel_signal,
            modules[1].drive_pos_signal, modules[1].drive_vel_signal, modules[1].turn_pos_signal, modules[1].turn_vel_signal,
            modules[2].drive_pos_signal, modules[2].drive_vel_signal, modules[2].turn_pos_signal, modules[2].turn_vel_signal,
            modules[3].drive_pos_signal, modules[3].drive_vel_signal, modules[3].turn_pos_signal, modules[3].turn_vel_signal
        };

        private final MedianFilter peak_remover = new MedianFilter(3);
        private final LinearFilter low_pass = LinearFilter.movingAverage(50);
        private double last_time = 0;
        private double current_time = 0;
        private double average_loop_time = 0;
        private int successful_daqs = 0;
        private int failed_daqs = 0;

        private int last_thread_priority = START_THREAD_PRIORITY;
        private volatile int thread_priority_to_set = START_THREAD_PRIORITY;

        private boolean isCollided;
        private double[] theoretical_acceleration;

        private final Debouncer collisDebouncer = new Debouncer(0.1,DebounceType.kBoth);

        private odometry_thread() {
            thread = new Thread(this::run);
            /* Mark this thread as a "daemon" (background) thread
                * so it doesn't hold up program shutdown */
            thread.setDaemon(true);
        }

        private void start() {
            is_running = true;
            thread.start();
        }

        private void stop() {
            stop(0);
        }

        private void stop(long millis) {
            is_running = false;
            try {
                thread.join(millis);
            } catch (final InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
        }

        private void run() {
            Preferences.initBoolean("swerve_optimization", true);
            Preferences.initDouble("skew_correction_sc", 0.0);
            BaseStatusSignal.setUpdateFrequencyForAll(odom_freq, all_signals);
            Threads.setCurrentThreadPriority(true, START_THREAD_PRIORITY);
            
            /* Run as fast as possible, our signals will control the timing */
            while (is_running) {
                if (!HootReplay.waitForPlaying(0.100)) {
                    continue;
                }

                /* Synchronously wait for all signals in drivetrain */
                /* Wait up to twice the period of the update frequency */
                StatusCode status;
                if (config.swerve.canivore) {
                    status = BaseStatusSignal.waitForAll(2.0 / odom_freq, all_signals);
                } else {
                    /* Wait for the signals to update */
                    Timer.delay(1.0 / odom_freq);
                    status = BaseStatusSignal.refreshAll(all_signals);
                }

                try {
                    state_lock.writeLock().lock();
                    input_lock.readLock().lock();

                    last_time = current_time;
                    current_time = Utils.getCurrentTimeSeconds();
                    /* We don't care about the peaks, as they correspond to GC events, and we want the period generally low passed */
                    average_loop_time = low_pass.calculate(peak_remover.calculate(current_time - last_time));

                    /* Get status of first element */
                    if (status.isOK()) {
                        successful_daqs++;
                    } else {
                        failed_daqs++;
                    }

                    /* Now update odometry */
                    /* Keep track of the change in azimuth rotations */
                    for (int i = 0; i < 4; ++i) {
                        modules[i].get_odom(module_positions[i]);
                        modules[i].get_state(current_states[i]);
                    }
                    var yaw = heading_signal.getValue();//BaseStatusSignal.getLatencyCompensatedValue(
                    //heading_signal, heading_rate_signal_odom);

                    Rotation2d pig_rotation = Rotation2d.fromDegrees(yaw.in(Degrees));

                    turret_pose_estimator.update(pig_rotation, module_positions);
                    pose = pose_estimator.update(pig_rotation, module_positions);
                    var heading = pose.getRotation();

                    var speeds = kin.to_chassis_speeds(current_states);

                    //collision detection

                    var output = commanded_field_relative_output.to_robot_relative( pig_rotation );

                    double y_accel = pig.getAccelerationY().getValue().in(MetersPerSecondPerSecond);
                    double x_accel = pig.getAccelerationX().getValue().in(MetersPerSecondPerSecond);

                    double collision_threshold = 7.0;

                    double[] theoretical_speeds = acceleration_estimate();

                    double xspeed_t = theoretical_speeds[0];
                    double yspeed_t = theoretical_speeds[1];

                    
                    double delta_x = Math.abs(x_accel - xspeed_t);
                    double delta_y = Math.abs(y_accel-yspeed_t);

                    //calculate the collision
                    double total_delta = Math.sqrt( delta_x * delta_x + delta_y * delta_y );
                    this.isCollided = collisDebouncer.calculate(total_delta > collision_threshold);
                    var field_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, heading);


                    current_field_relative_speeds.vxMetersPerSecond = field_speeds.vxMetersPerSecond;
                    current_field_relative_speeds.vyMetersPerSecond = field_speeds.vyMetersPerSecond;
                    current_field_relative_speeds.omegaRadiansPerSecond = field_speeds.omegaRadiansPerSecond;


                    // theoretical speeds that are public for everyone to enjoy
                    this.theoretical_acceleration = theoretical_speeds;

                    //send estimated acc
                    // handle_vision(heading);


                    ctre_log_pose("fused_pose", pose);

                    if(burning_motors() || request.stop_motors) {
                        modules[0].stop();
                        modules[1].stop();
                        modules[2].stop();
                        modules[3].stop();
                    } else {

                        if(DriverStation.isDisabled()) {
                            request.with_chassis_output(0, 0, 0);
                        }

                        // kin.clamp(request.target_field_relative, request.turn_bias);
                        // var optimized = request.target_field_relative;
                        // if(Preferences.getBoolean("swerve_optimization", true)) {
                        //     optimized = optimizer.optimize(current_field_relative_speeds, request.target_field_relative, request.max_strafe_torque, request.max_slip_angle);
                        // }

                        // var desired_chassis_output = new chassis_output(ChassisSpeeds.fromFieldRelativeSpeeds(optimized.as_speeds(), heading))
                        //     .deadbanded(strafe_deadzone, omega_deadzone);

                        var target_out = request.target_field_relative.copy();
                        kin.clamp(target_out, request.turn_bias);
                        // target_out = torque_limiting.limit_torque( target_out, current_field_relative_speeds,
                        //     constants.swerve.max_module_speed_mps, request.max_strafe_torque, request.max_slip_angle );

                        var desired_chassis_output = new chassis_output(ChassisSpeeds.fromFieldRelativeSpeeds(target_out.as_speeds(), heading))
                            .deadbanded(strafe_deadzone, omega_deadzone);

                        //
                        // chassis-relative anti tip
                        //
                        desired_chassis_output.x_output = torque_limiting.limit_torque_asym(
                            desired_chassis_output.x_output, speeds.vxMetersPerSecond, max_module_speed_mps, 
                            request.max_strafe_torque, 0.35 * request.max_strafe_torque
                        );
                        desired_chassis_output.y_output = torque_limiting.limit_torque_asym(
                            desired_chassis_output.y_output, speeds.vyMetersPerSecond, max_module_speed_mps, 
                            request.max_strafe_torque, 0.35 * request.max_strafe_torque
                        );

                        // Antiskew
                        double sc = config.swerve.skew_correction;
                        double px = desired_chassis_output.y_output;
                        double py = -desired_chassis_output.x_output;
                        desired_chassis_output.x_output += px * desired_chassis_output.omega_output * sc;
                        desired_chassis_output.y_output += py * desired_chassis_output.omega_output * sc;

                        SmartDashboard.putNumber("swerve_magnitude", desired_chassis_output.magnitude());
                        SmartDashboard.putNumber("swerve_turn_output", desired_chassis_output.omega_output);
                        if(desired_chassis_output.magnitude_squared() >= strafe_deadzone * strafe_deadzone || Math.abs(desired_chassis_output.omega_output) >= omega_deadzone) {
                            kin.to_module_states(desired_chassis_output, desired_states);
                            kin.desaturateWheelSpeeds(desired_states);
                        } else if(request.form_x_when_stopped) {
                            for(int i = 0; i < 4; ++i) {
                                desired_states[i].drive_output = form_x_states[i].drive_output;
                                desired_states[i].theta_rad = form_x_states[i].theta_rad;
                                desired_states[i].omega_radps = form_x_states[i].omega_radps;
                            }
                        } else {
                            desired_states[0].drive_output = 0;
                            desired_states[1].drive_output = 0;
                            desired_states[2].drive_output = 0;
                            desired_states[3].drive_output = 0;
                        }

                        modules[0].set_state(desired_states[0]);
                        modules[1].set_state(desired_states[1]);
                        modules[2].set_state(desired_states[2]);
                        modules[3].set_state(desired_states[3]);
                    }


                    // swerve_module.set_state() can modify the desired state with cosine scale and optimize
                    // best to put the commanded chassis speeds calculation after the set_state()
                    var out = kin.to_chassis_output(desired_states).to_field_relative(heading);
                    commanded_field_relative_output.x_output = out.x_output;
                    commanded_field_relative_output.y_output = out.y_output;
                    commanded_field_relative_output.omega_output = out.omega_output;

                    pig.getSimState().addYaw(Radians.of(speeds.omegaRadiansPerSecond * odom_dts));
                    pig.getSimState().setAngularVelocityZ(RadiansPerSecond.of(speeds.omegaRadiansPerSecond));

                } finally {
                    state_lock.writeLock().unlock();
                    input_lock.readLock().unlock();
                }

                /**
                 * This is inherently synchronous, since lastThreadPriority
                 * is only written here and threadPriorityToSet is only read here
                 */
                if (thread_priority_to_set != last_thread_priority) {
                    Threads.setCurrentThreadPriority(true, thread_priority_to_set);
                    last_thread_priority = thread_priority_to_set;
                }
            }
        }

        public boolean odometryIsValid() {
            return successful_daqs > 2; // Wait at least 3 daqs before saying the odometry is valid
        }

        /**
         * Sets the DAQ thread priority to a real time priority under the specified priority level
         *
         * @param priority Priority level to set the DAQ thread to.
         *                 This is a value between 0 and 99, with 99 indicating higher priority and 0 indicating lower priority.
         */
        public void setThreadPriority(int priority) {
            thread_priority_to_set = priority;
        }
    }

    public swerve_lowlevel(String canbus, module_config[] module_configs, TimedRobot robot) {
        modules = new swerve_module[]{
            new swerve_module(module_configs[0], canbus, robot),
            new swerve_module(module_configs[1], canbus, robot),
            new swerve_module(module_configs[2], canbus, robot),
            new swerve_module(module_configs[3], canbus, robot),
        };
        heading_signal = pig.getYaw();
        heading_rate_signal_odom = pig.getAngularVelocityZWorld();
        heading_rate_signal = pig.getAngularVelocityZWorld();
        pitch_signal = pig.getPitch();

        pose_estimator = new SwerveDrivePoseEstimator(kin, pig.getRotation2d(), module_positions, pose);
        turret_pose_estimator = new SwerveDrivePoseEstimator(kin, pig.getRotation2d(), module_positions, pose);

        robot.addPeriodic(this::nt_periodic, RobotBase.isSimulation() ? 1.0/100.0 : 1.0/16.0, 1.0/32.0);
        odom_thread = new odometry_thread();
        odom_thread.start();

        SmartDashboard.putData(field);
    }

    private void nt_periodic() {
        
        try {
            state_lock.readLock().lock();

            field.setRobotPose(pose);

            SmartDashboard.putBoolean("toasty warning", !toasty_motors());
            SmartDashboard.putBoolean("burning motors", burning_motors());
            swerve_vis.update(get_heading(), desired_states, current_states);
            if(config.dev) {
                var speeds = current_field_relative_speeds;
                SmartDashboard.putNumber("swerve/vx", speeds.vxMetersPerSecond);
                SmartDashboard.putNumber("swerve/vy", speeds.vyMetersPerSecond);
                SmartDashboard.putNumber("swerve/vt", speeds.omegaRadiansPerSecond);
                SmartDashboard.putNumber("swerve/vhypot", math_utils.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
                SmartDashboard.putNumber("swerve/x", pose.getX());
                SmartDashboard.putNumber("swerve/theta", get_heading().getRadians());
            }
        } finally {
            state_lock.readLock().unlock();
        }

    }

    public void add_vision_measurement(Pose2d pose, double timestamp, Matrix<N3, N1> st_devs) {
        try {
            state_lock.writeLock().lock();
            pose_estimator.addVisionMeasurement(pose, timestamp, st_devs);
        } finally {
            state_lock.writeLock().unlock();
        }
    }

    public void add_turret_vision_measurement( Translation2d pose ) {
        try {
            state_lock.writeLock().lock();
            turret_pose_estimator.resetTranslation( pose );
        } finally {
            state_lock.writeLock().unlock();
        }
    }

    public Translation2d get_turret_pose() {
        try {
            state_lock.readLock().lock();
            return turret_pose_estimator.getEstimatedPosition().getTranslation();
        } finally {
            state_lock.readLock().unlock();
        }
    }

    public double get_max_chassis_radps() {
        return constants.swerve.max_module_speed_mps / kin.chassis_radius;
    }

    public double chassis_radius(int module) {
        return kin.module_mount_positions[module].getNorm();
    }

    public void set_strafe_lowlevel(chassis_output output) {
        try {
            input_lock.writeLock().lock();
            request.target_field_relative.x_output = output.x_output;
            request.target_field_relative.y_output = output.y_output;
        } finally {
            input_lock.writeLock().unlock();
        }
    }

    public void set_omega_lowlevel(double omega_output) {
        try {
            input_lock.writeLock().lock();
            request.target_field_relative.omega_output = omega_output;
        } finally {
            input_lock.writeLock().unlock();
        }
    }

    public void set_swerve_lowlevel(chassis_output output) {
        try {
            input_lock.writeLock().lock();
            request = default_request;
            request.target_field_relative.x_output = output.x_output;
            request.target_field_relative.y_output = output.y_output;
            request.target_field_relative.omega_output = output.omega_output;
        } finally {
            input_lock.writeLock().unlock();
        }
    }

    public void set_swerve_lowlevel(swerve_request request) {
        if(request == null) {
            System.err.println("swerve request was null, setting static brake instead");
            request = default_static_brake;
            return;
        }
        try {
            input_lock.writeLock().lock();
            this.request = request;
        } finally {
            input_lock.writeLock().unlock();
        }
    }

    public Angle get_pitch() {
        return pitch_signal.refresh().getValue();
    }

    public void coast() {
        modules[0].coast();
        modules[1].coast();
        modules[2].coast();
        modules[3].coast();
    }

    public void brake() {
        modules[0].brake();
        modules[1].brake();
        modules[2].brake();
        modules[3].brake();
    }

    public boolean toasty_motors() {
        return modules[0].toasty() 
            || modules[1].toasty() 
            || modules[2].toasty()
            || modules[3].toasty();
    }

    public boolean burning_motors() {
        return modules[0].burning()
            || modules[1].burning()
            || modules[2].burning()
            || modules[3].burning();
    }

    public SwerveModulePosition[] copy_module_positions() {
        final int length = module_positions.length;
        SwerveModulePosition[] copy = new SwerveModulePosition[length];
        for(int i = 0; i < length; ++i) {
            copy[i] = new SwerveModulePosition(module_positions[i].distanceMeters, Rotation2d.fromRadians(module_positions[i].angle.getRadians()));
        }
        return copy;
    }

    public chassis_output get_commanded_speeds() {
        try {
            state_lock.readLock().lock();
            return new chassis_output(
                commanded_field_relative_output.x_output,
                commanded_field_relative_output.y_output,
                commanded_field_relative_output.omega_output
            );
        } finally {
            state_lock.readLock().unlock();
        }
    }

    public void zero_heading(Rotation2d new_heading) {
        reset_pose(new Pose2d(get_pose().getTranslation(), new_heading));
    }

    public void reset_pose(Pose2d pose) {
        try {
            state_lock.writeLock().lock();
            pose_estimator.resetPosition(pig.getRotation2d(), module_positions, pose);
            turret_pose_estimator.resetPosition(pig.getRotation2d(), module_positions, pose);
            this.pose = pose;
        } finally {
            state_lock.writeLock().unlock();
        }
    }

    public void reset_pose(Translation2d pose) {
        reset_pose(new Pose2d(pose, get_heading()));
    }

    public Pose2d get_pose() {
        try {
            state_lock.readLock().lock();
            return new Pose2d(pose.getX(), pose.getY(), pose.getRotation().times(1));
        } finally {
            state_lock.readLock().unlock();
        }
    }
    //acceleration - collision_detection
    public double[] acceleration_estimate() {
        try {
            state_lock.readLock().lock();
            return torque_limiting.estimate_acceleration(commanded_field_relative_output, current_field_relative_speeds, constants.swerve.max_speed_mps);
        } finally {
            state_lock.readLock().unlock();
        }
    }

    protected static void ctre_log_pose(String name, Pose2d pose) {
        SignalLogger.writeDoubleArray(name, new double[]{pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
    }
    protected static void ctre_log_pose(String name, Translation2d pose) {
        SignalLogger.writeDoubleArray(name, new double[]{pose.getX(), pose.getY(), 0.0});
    }
    protected static void ctre_log_pose(String name, Translation2d pose, double angle) {
        SignalLogger.writeDoubleArray(name, new double[]{pose.getX(), pose.getY(), angle});
    }

    public Rotation2d get_heading() {
        return get_pose().getRotation();
    }

    public AngularVelocity get_heading_rate() {
        return heading_rate_signal.refresh().getValue();
    }

    public Rotation2d get_raw_pigeon_rotation() {
        return pig.getRotation2d();
    }

    public ChassisSpeeds get_field_relative_speeds() {
        try {
            state_lock.readLock().lock();
            return new ChassisSpeeds(
                current_field_relative_speeds.vxMetersPerSecond,
                current_field_relative_speeds.vyMetersPerSecond,
                current_field_relative_speeds.omegaRadiansPerSecond
            );
        } finally {
            state_lock.readLock().unlock();
        }
    }

    public ChassisSpeeds get_speeds() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(get_field_relative_speeds(), get_heading());
    }

    public boolean is_auto_limits = true;
    public void apply_current_limits( boolean auto ) {
        if( auto != is_auto_limits ) {
            modules[0].apply_limits(auto);
            modules[1].apply_limits(auto);
            modules[2].apply_limits(auto);
            modules[3].apply_limits(auto);
            is_auto_limits = auto;
        }
    }

    @Override
    public void configure() {
        modules[0].configure();
        modules[1].configure();
        modules[2].configure();
        modules[3].configure();
        pig.getConfigurator().apply(config.swerve.pig_config);
    }
    
}