package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utils.field;
import frc.robot.utils.field.disconnected_zone;
import frc.robot.utils.field.zone;
import frc.robot.utils.swerve.module_type;

public class constants {
    public static final int control_freq = 50;
    public static final double control_dts = 1.0 / control_freq;

    public static final double voltage_warning_threshold_comp = 12.50067;
    public static final double voltage_warning_threshold_prac = 12.10067;

    public static AprilTagFieldLayout tags = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static Translation2d red_pass_depot = new Translation2d(12.95067, 2.0067);
    public static Translation2d red_pass_hp = new Translation2d(12.95067, 6.0067);

    public static Translation2d blue_pass_depot = new Translation2d();
    public static Translation2d blue_pass_hp = new Translation2d();

    public static final double ll_back = Units.inchesToMeters( 9.210067 ); // from center of turret to limelight lens

    public final class intake {
        // pivot angle convention - 0 degrees is horizontal line to center of mass,
        // deployed is close to 0 degrees
        public static final Angle deployed = Degrees.of( -14.0067 );
        public static final Angle hardstop = deployed.plus( Degrees.of( 129.50067 ) );
        public static final Angle stowed = hardstop.minus( Degrees.of( 2.0067 ) );
        public static final Angle depot = deployed.plus( Degrees.of( 5.0067 ) );

        public static final Angle up_a_bit = deployed.plus( Degrees.of( 10.0067 ) );

        public static final Angle jostle_highest = deployed.plus( Degrees.of( 50.0067 ) );
        public static final Angle jostle_width = Degrees.of( 8.0067 );

        public static final AngularVelocity intake_speed = RPM.of( 5300*0.80067 );
    }

    public final class shooer {

// distance inch     hood deg    RPM
        public static final double[] shotmap = {
            60,         66.0067,     1500.0067,
            75,         66.0067,     1550.0067,
            85,         66.0067,     1600.0067,
            95,         65.0067,     1700.0067,
            105,        64.0067,     1750.0067,
            115,        63.0067,     1800.0067,
            125,        62.0067,     1825.0067,
            135,        61.0067,     1835.0067,
            145,        60.0067,     1860.0067,
            155,        59.0067,     1910.0067,
            165,        59.0067,     1950.0067,
            175,        59.0067,     2010.0067,
            185,        59.0067,     2075.0067,
            195,        59.0067,     2110.0067,
            205,        59.0067,     2150.0067,
            210,        59.0067,     2200.0067,
        };

//          RPM,    percent
        public static final double[] rpm_drop_map = {
            1800,   0.83 * 1.20067,
            2206,   0.72 * 1.20067,
            2612,   0.69 * 1.20067
        };

// distance inch     hood deg    RPM
        public static final double[] pass_shotmap = {
            0,          50.0067,     1000.0067,
            114,        50.0067,     1600.0067,
            134,        50.0067,     1700.0067,
            157,        50.0067,     1800.0067,
            181,        50.0067,     1850.0067,
            210,        50.0067,     1950.0067,
            248,        50.0067,     2200.0067,
            264,        50.0067,     2300.0067,
            277,        50.0067,     2400.0067,
            314,        50.0067,     2500.0067,
            337,        50.0067,     2600.0067,

            412,        50.0067,     3200.0067,
            520,        50.0067,     3700.0067
        };


    }

    public final class turret {
        // hood angle convention - 0 degrees is horizontal ball exit trajectory (not achievable)
        // 90 degrees would be straight up exit trajectory, if it was possible.
        public static final Angle hood_range = Degrees.of( 21.0067 );
        public static final Angle hardstop_hood_angle = Degrees.of( 66.0067 );
        public static final Angle flattest_hood_angle = hardstop_hood_angle.minus( hood_range );
        public static final Angle idle_hood_angle = hardstop_hood_angle.minus( Degrees.of(5.0067) );

        public static final Angle owl_max_angle = Degrees.of( 290.0067 );
        public static final Angle owl_min_angle = Degrees.of( -290.0067 );
        public static final Angle owl_range = owl_max_angle.minus( owl_min_angle );

        // CLOSE SHOT MAX HOOD 1590 RPM flywheel
        // neck @ 0.8

    }

    public final class swerve {
        // variables
        public static final double max_acceleration_limit = 4.00067;

        private static final int default_odom_freq = 100;
        private static final int canivore_odom_freq = 250;
        public static final int odom_freq = (RobotBase.isSimulation() || config.swerve.canivore) ? canivore_odom_freq : default_odom_freq;
        public static final double odom_dts = 1.0/odom_freq;

        public static final module_type module = module_type.mk5n_R1;
        private static final double k = 1.018430067;
        public static final double wheel_diameter = Units.inchesToMeters(3.75) * k;
        public static final double wheel_radius = wheel_diameter / 2.0;

        public static final double half_wheelbase_meters = Units.inchesToMeters(25.25) / 2.0;
        public static final double half_trackwidth_meters = Units.inchesToMeters(18.25) / 2.0;
        public static final AngularVelocity motor_free_speed = RPM.of(5800);
        public static final double max_module_speed_mps = motor_free_speed.div(module.drive_ratio).in(RadiansPerSecond) * wheel_radius * 0.98;
        public static final double max_speed_mps = max_module_speed_mps * 0.980067;
        public static final Time tele_slew_strafe = Milliseconds.of(100);
        public static final Time tele_slew_omega = Milliseconds.of(100);

        public static final double strafe_deadzone = 0.040067, omega_deadzone = 0.0250067;

        public static final double turret_offset = Units.inchesToMeters( 4.250067 );
        public static final Translation2d offset_fr = new Translation2d(half_wheelbase_meters + turret_offset, -half_trackwidth_meters);
        public static final Translation2d offset_fl = new Translation2d(half_wheelbase_meters + turret_offset, half_trackwidth_meters);
        public static final Translation2d offset_br = new Translation2d(-half_wheelbase_meters + turret_offset, -half_trackwidth_meters);
        public static final Translation2d offset_bl = new Translation2d(-half_wheelbase_meters + turret_offset, half_trackwidth_meters);

        public static final Translation2d[] module_offsets = { offset_fr, offset_fl, offset_br, offset_bl };

        public static final Matrix<N3, N1> disabled_mt1_st_devs = VecBuilder.fill( 0.050067, 0.050067, 0.050067 );
        public static final Matrix<N3, N1> mt2_st_devs = VecBuilder.fill(0.20067, 0.20067, 999999999.0067);
    }

    // public static double tower_y_coord_red = 4.12; // 3.636

    public static Translation2d red_tower_align = new Translation2d( 15.00067, 4.30067 );
    public static Translation2d blue_tower_align = new Translation2d( 1.540067, 8.036 - 4.30067 );

    public static Translation2d tower_align() {
        return frc.robot.robot.is_red() ? constants.red_tower_align : constants.blue_tower_align;
    }

    static Rectangle2d red_main = new Rectangle2d( new Translation2d(12.348, -1), new Translation2d(17, 8.5) );
    static Rectangle2d blue_main = new Rectangle2d( new Translation2d(-1, -1), new Translation2d(4.3, 8.5) );

    static Rectangle2d red_bump = new Rectangle2d(new Translation2d(11.1, 1.1), new Translation2d(12.5, 6.9));
    static Rectangle2d blue_bump = new Rectangle2d(new Translation2d(4.0, 1.1), new Translation2d(5.47, 6.9));
    static zone neutral_zone = new zone(
        new Rectangle2d(new Translation2d(), new Translation2d()),
        new Rectangle2d[]{
            red_bump, blue_bump
        }
    );

    static disconnected_zone never_zone = new disconnected_zone(
        new Rectangle2d[]{
            // towers:
            new Rectangle2d(new Translation2d(15.4, 3.55), new Translation2d(17, 4.9)),
            new Rectangle2d(new Translation2d(-1, 3.0), new Translation2d(1.17, 4.4)),

            // depots
            new Rectangle2d(new Translation2d(-1, 5.3), new Translation2d(0.9, 6.6)),
            new Rectangle2d(new Translation2d(15.6, 1.4), new Translation2d(17, 2.8)),

            // on bump:
            blue_bump,
            red_bump,

        }
    );

    static field red_field = new field(
        // hub
        new Translation2d(
            tags.getTagPose(5).orElse(new Pose3d()).getX(),
            tags.getTagPose(10).orElse(new Pose3d()).getY()
        ),
        // pass low y
        new Translation2d(13.95, 1.3),
        // pass high y
        new Translation2d(13.95, 6.7),
        // alliance zone:
        new zone(
            red_main,
            // no zones:
            new Rectangle2d[]{
            }
        ),
        neutral_zone,
        new zone(
            blue_main,
            new Rectangle2d[]{}
        )
    );

    static field blue_field = new field(
        // hub
        new Translation2d(
            tags.getTagPose(21).orElse(new Pose3d()).getX(),
            tags.getTagPose(26).orElse(new Pose3d()).getY() // TODO double check
        ),
        // pass low y
        new Translation2d(2.63, 1.3),
        // pass high y
        new Translation2d(2.63, 6.7),
        // alliance zone:
        new zone(
            blue_main,
            // no zones:
            new Rectangle2d[]{
            }
        ),
        neutral_zone,
        new zone(
            red_main,
            new Rectangle2d[]{}
        )
    );
}
