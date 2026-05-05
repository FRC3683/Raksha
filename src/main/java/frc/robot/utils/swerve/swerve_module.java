package frc.robot.utils.swerve;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.constants.swerve.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config;
import frc.robot.constants;
import frc.robot.utils.can_debugger;
import frc.robot.utils.configurable;
import frc.robot.utils.lazy_ctre;
import frc.robot.utils.talon_temps_safety;
import frc.robot.utils.controls.dave_talon;
import frc.robot.utils.swerve.swerve_kin2.module_output;
import frc.robot.utils.swerve.swerve_kin2.module_speed;

public class swerve_module implements configurable {

    public static final class module_config {
        public final String name;
        public final int can_drive, can_turn, can_coder;
        public final InvertedValue drive_inverted;
        public final InvertedValue turn_inverted;
        public final double comp_abs_offset, prac_abs_offset;

        public module_config(String name, int can_drive, int can_turn, int can_coder, 
                InvertedValue drive_inverted, InvertedValue turn_inverted, 
                double comp_abs_offset, double prac_abs_offset) {
            this.name = name;
            this.can_drive = can_drive;
            this.can_turn = can_turn;
            this.can_coder = can_coder;
            this.drive_inverted = drive_inverted;
            this.turn_inverted = turn_inverted;
            this.comp_abs_offset = comp_abs_offset;
            this.prac_abs_offset = prac_abs_offset;
        }
    }

    public final String nt_name;
    public final talon_temps_safety drive_temps, turn_temps;
    private final dave_talon drive, turn;
    private final CANcoder cancoder;
    public final StatusSignal<Angle> drive_pos_signal, turn_pos_signal;
    public final StatusSignal<AngularVelocity> drive_vel_signal, turn_vel_signal;
    private final PositionVoltage turn_control_out = new PositionVoltage(0).withEnableFOC(config.swerve.ctre_pro).withSlot(0);
    private final VoltageOut drive_control_raw_out = new VoltageOut(0).withEnableFOC(config.swerve.ctre_pro);
    private final module_config module_config;
    private final double couple_ratio;

    private double last_output, last_ang;
    private module_output applied_state = new module_output(0, 0, 0);

    private final MotorOutputConfigs drive_output_configs, turn_output_configs;

    public swerve_module(module_config module_config, String canbus, TimedRobot robot) {
        drive = new dave_talon(module_config.can_drive, canbus);
        turn = new dave_talon(module_config.can_turn, canbus);
        cancoder = new CANcoder(module_config.can_coder, canbus);

        drive_output_configs = config.swerve.drive_configs(module_config.drive_inverted).MotorOutput;
        turn_output_configs = config.swerve.turn_configs(module_config.drive_inverted, cancoder).MotorOutput;
        this.nt_name = module_config.name;
        this.module_config = module_config;
        double hard_coded_offset = config.is_comp ? module_config.comp_abs_offset : module_config.prac_abs_offset;
        couple_ratio = RobotBase.isReal() ? module.couple_ratio : 0;

        can_debugger.add_talons(drive, turn);

        drive_pos_signal = drive.getPosition();
        drive_vel_signal = drive.getVelocity();
        turn_pos_signal = turn.getPosition();
        turn_vel_signal = turn.getVelocity();

        turn.setSafetyEnabled(false);
        drive.setSafetyEnabled(false);

        BaseStatusSignal.setUpdateFrequencyForAll(constants.swerve.odom_freq,
            drive.getMotorVoltage()
        );

        BaseStatusSignal.setUpdateFrequencyForAll(4,
            turn.getClosedLoopReference()
        );

        drive_temps = new talon_temps_safety(drive, "swerve/"+nt_name+"_drive_", Celsius.of(70), Celsius.of(85));
        turn_temps = new talon_temps_safety(turn, "swerve/"+nt_name+"_turn_", Celsius.of(70), Celsius.of(85));

        ParentDevice.optimizeBusUtilizationForAll(drive, turn);

        robot.addPeriodic(this::periodic, RobotBase.isReal() ? 0.25 : 0.02, 0.125); // 4hz, 125ms offset
        if(RobotBase.isSimulation()) {
            turn.init_sim(robot, DCMotor.getKrakenX60Foc(1), module.steer_ratio, 0.001);
            drive.init_sim(robot, DCMotor.getKrakenX60Foc(1), module.drive_ratio, 0.04);
        }
    }

    private void periodic() {
        var base = "swerve/"+nt_name+"_";
        if(config.dev) {
            SmartDashboard.putNumber(base+"position_m", get_drive_position_m());
            SmartDashboard.putNumber(base+"coupling", get_drive_position().in(Rotations) / get_turning_position().in(Rotations));
            SmartDashboard.putNumber(base+"desired_mps", applied_state.drive_output);
        }
        drive_temps.periodic();
        turn_temps.periodic();
    }

    public boolean toasty() {
        return turn_temps.toasty() || drive_temps.toasty();
    }

    public boolean burning() {
        return turn_temps.cutoff() || drive_temps.cutoff();
    }

    public Angle get_turning_position() {
        return BaseStatusSignal.getLatencyCompensatedValue(turn_pos_signal, turn_vel_signal);
    }

    public AngularVelocity get_turning_velocity() {
        return turn_vel_signal.getValue();
    }

    public Angle get_drive_position() {
        return BaseStatusSignal.getLatencyCompensatedValue(drive_pos_signal, drive_vel_signal).plus((get_turning_position().times(couple_ratio)));
    }

    public double get_drive_position_m() {
        return get_drive_position().in(Radians) * constants.swerve.wheel_radius;
    }

    public AngularVelocity get_drive_velocity() {
        return drive_vel_signal.getValue().plus(get_turning_velocity().times(couple_ratio));
    }

    public double get_drive_velocity_mps() {
        return get_drive_velocity().in(RadiansPerSecond) * constants.swerve.wheel_radius;
    }

    public void get_odom(SwerveModulePosition position) {
        position.angle = Rotation2d.fromRotations(get_turning_position().in(Rotations));
        position.distanceMeters = get_drive_position_m();
    }

    public void get_state(module_speed current_state) {
        current_state.drive_speed_mps = get_drive_velocity_mps();
        current_state.theta_rad = get_turning_position().in(Radians);
        current_state.omega_radps = get_turning_velocity().in(RadiansPerSecond);
    }

    private void set_speed(double output) {
        if(last_output == output) {
            return;
        }
        last_output = output;
        drive.setControl(drive_control_raw_out.withOutput(output));
    }

    private void set_turn(double theta_rad, double omega_radps) {
        if(last_ang == theta_rad) {
            return;
        }
        last_ang = theta_rad;
        turn.setControl(turn_control_out.withPosition(theta_rad / (Math.PI * 2.0)).withVelocity(omega_radps / (Math.PI * 2.0)));
    }

    public void stop() {
        drive.stopMotor();
        turn.stopMotor();
    }

    public void set_state(module_output desired_state) {
        desired_state.optimize(get_turning_position());

        if(Math.abs(desired_state.drive_output) > 0.002) {
            set_turn(desired_state.theta_rad, desired_state.omega_radps);
        }

        /* From FRC 900's whitepaper, we add a cosine compensator to the applied drive velocity */
        /* To reduce the "skew" that occurs when changing direction */
        double azimuth_err = desired_state.theta_rad - get_turning_position().in(Radians);
        /* If error is close to 0 rotations, we're already there, so apply full power */
        /* If the error is close to 0.25 rotations, then we're 90 degrees, so movement doesn't help us at all */
        double cosine_scalar = Math.cos(azimuth_err);
        /* Make sure we don't invert our drive, even though we shouldn't ever target over 90 degrees anyway */
        if(cosine_scalar < 0.0) {
            cosine_scalar = 0.0;
        }

        desired_state.drive_output *= cosine_scalar; // TODO test cube_scalar on real chassis

        /* Back out the expected shimmy the drive motor will see */
        /* Find the angular rate to determine what to back out */
        double azimuth_turn_rotations_ps = get_turning_velocity().in(RotationsPerSecond);
        /* Azimuth turn rate multiplied by coupling ratio provides back-out rps */
        double drive_rate_back_out = azimuth_turn_rotations_ps * couple_ratio * config.swerve.coupling_kV;

        set_speed(desired_state.drive_output * 12.0 - drive_rate_back_out);

        applied_state = desired_state;
    }

    public void coast() {
        drive.getConfigurator().apply(drive_output_configs.withNeutralMode(NeutralModeValue.Coast));
        turn.getConfigurator().apply(turn_output_configs.withNeutralMode(NeutralModeValue.Coast));
    }

    public void brake() {
        drive.getConfigurator().apply(drive_output_configs.withNeutralMode(NeutralModeValue.Brake));
        turn.getConfigurator().apply(turn_output_configs.withNeutralMode(NeutralModeValue.Brake));
    }

    public void apply_limits( boolean auto ) {
        drive.getConfigurator().apply( auto ? config.swerve.auto_limits : config.swerve.tele_limits );
    }

    @Override
    public void configure() {
        drive.clearStickyFaults();
        lazy_ctre.lazy_config(drive, config.swerve.drive_configs(module_config.drive_inverted));
        turn.clearStickyFaults();
        lazy_ctre.lazy_config(turn, config.swerve.turn_configs(module_config.turn_inverted, cancoder));
    }
}
