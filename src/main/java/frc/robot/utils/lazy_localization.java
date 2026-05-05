package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.LimelightHelpers;
import frc.robot.constants;

public final class lazy_localization {
    private lazy_localization() {}

    public static final Optional<Translation2d> get_camera_pos() {
        var estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2( "limelight-turret" );
        if( estimate != null ) {
            if( estimate.tagCount < 2 ) {
                return Optional.empty();
            }
            Translation2d us = estimate.pose.getTranslation();
            if( us.getX() < 0.1 || us.getX() > constants.tags.getFieldLength() - 0.1 
            || us.getY() < 0.1 || us.getY() > constants.tags.getFieldWidth() - 0.1 ) {
                // outside of field, ignore
                return Optional.empty();
            }
            return Optional.of( us );
        }
        return Optional.empty();
    }

    public static final Optional<Translation2d> get_turret_pos() {
        Optional<Translation2d> camera = get_camera_pos();
        if( camera.isEmpty() ) {
            return Optional.empty();
        }
        Translation2d camera_pos = camera.get();
        Translation2d turret_pos = camera_pos;
        return Optional.of( turret_pos );
    }
}
