package frc.robot.utils;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.IMUData;
import frc.robot.LimelightHelpers.RawFiducial;

public final class localization {

    private static AprilTagFieldLayout layout = AprilTagFieldLayout.loadField( AprilTagFields.kDefaultField );

    public static void init() {
        layout.getTags(); // force lazy static initialization
    }

    private static Set<Integer> red_hub_tags = Set.of( 5, 8, 9, 10, 11, 2 );
    private static Set<Integer> blue_hub_tags = Set.of( 21, 24, 25, 26, 27, 18 );

    private static final double hub_tags_height = Units.inchesToMeters( 44.25 );
    private static final double ll_mount_height = Units.inchesToMeters( 28.5 );
    private static final double delta_h = hub_tags_height - ll_mount_height;

    private static final double ll_back = Units.inchesToMeters( 9.21 );

    private static Optional<Translation2d> intersection( Translation2d p1, Rotation2d theta1, Translation2d p2, Rotation2d theta2 ) {
        Translation2d l1 = new Translation2d( 1, theta1 );
        Translation2d l2 = new Translation2d( 1, theta2 );

        // Line 1
        double x1 = p1.getX(), y1 = p1.getY(), x2 = x1 + l1.getX(), y2 = y1 + l1.getY();
        // Line 2
        double x3 = p2.getX(), y3 = p2.getY(), x4 = x3 + l2.getX(), y4 = y3 + l2.getY();

        double a1 = y2 - y1;
        double b1 = x1 - x2;
        double c1 = a1 * x1 + b1 * y1;

        double a2 = y4 - y3;
        double b2 = x3 - x4;
        double c2 = a2 * x3 + b2 * y3;

        double delta = a1 * b2 - a2 * b1;

        // If delta is 0, the lines are parallel or coincident
        if (delta == 0) {
            return Optional.empty();
        }

        // Calculate the intersection point coordinates
        double x = (b2 * c1 - b1 * c2) / delta;
        double y = (a1 * c2 - a2 * c1) / delta;

        return Optional.of( new Translation2d( x, y ) );
    }

    private static Optional<Translation2d> get_lens_position( String name, Alliance alliance, RawFiducial[] seen_tags, IMUData imu_data ) {
        Set<Integer> hub_tags = ( alliance == Alliance.Blue ) ? blue_hub_tags : red_hub_tags;

        List<RawFiducial> good_tags = Arrays.stream( seen_tags ).filter( (RawFiducial tag) -> hub_tags.contains( tag.id ) ).toList();

        if( good_tags.size() == 0 ) {
            return Optional.empty();
        }

        //
        // SINGLE TAG CASE
        //
        if( good_tags.size() == 1 ) {
            RawFiducial tag = seen_tags[0];
            Optional<Pose3d> tag_position = layout.getTagPose( tag.id );
            if( tag_position.isEmpty() ) {
                return Optional.empty();
            }
            Translation2d tag_2d = tag_position.get().toPose2d().getTranslation();
            double mount_angle_deg = 2;
            double alpha = Units.degreesToRadians( tag.tync + mount_angle_deg );
            double distance = delta_h / Math.atan( alpha );
            double heading_to_tag = Units.degreesToRadians( imu_data.robotYaw - tag.txnc );
            Translation2d camera_to_tag = new Translation2d( distance, Rotation2d.fromRadians(heading_to_tag) );
            return Optional.of( tag_2d.minus( camera_to_tag ) );
        }

        //
        // GET WIDEST SPLIT
        //
        RawFiducial leftmost = good_tags.get(0);
        RawFiducial rightmost = good_tags.get(1);
        for( RawFiducial tag : good_tags ) {
            if( tag.txnc < leftmost.txnc ) {
                leftmost = tag;
            }
            if( tag.txnc > rightmost.txnc ) {
                rightmost = tag;
            }
        }
        Optional<Pose3d> left = layout.getTagPose( leftmost.id );
        Optional<Pose3d> right = layout.getTagPose( rightmost.id );
        if( left.isEmpty() || right.isEmpty() ) {
            return Optional.empty();
        }

        //
        // TWO TAGS CASE
        //
        double heading_to_left = imu_data.robotYaw - leftmost.txnc;
        double heading_to_right = imu_data.robotYaw - rightmost.txnc;

        return intersection(
            left.get().toPose2d().getTranslation() , Rotation2d.fromDegrees( heading_to_left ),
            right.get().toPose2d().getTranslation(), Rotation2d.fromDegrees( heading_to_right )
        );
    }

    public static Optional<Translation2d> get_turret_position( String name, Alliance alliance ) {
        // time synced?
        RawFiducial[] seen_tags = LimelightHelpers.getRawFiducials( name );
        IMUData imu_data = LimelightHelpers.getIMUData( name );

        Optional<Translation2d> lens = get_lens_position( name, alliance, seen_tags, imu_data );
        if( lens.isEmpty() ) {
            return Optional.empty();
        }

        return Optional.of( lens.get().plus( new Translation2d( ll_back, Rotation2d.fromDegrees( imu_data.robotYaw + 180 ) ) ) ); 
    }
}
