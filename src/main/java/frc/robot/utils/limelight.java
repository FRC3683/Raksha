package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.robot;

public class limelight {
    public Supplier<Translation2d> coordinateVector;

    String limelight_name;
    double dist_between_two_targets;

    Mechanism2d three_tag_case_mechanism = new Mechanism2d(16.54, 8);
    RawFiducial[] fiducials;
    HashMap<Double, Double> dists_from_targets = new HashMap<Double, Double>();
    AprilTagFieldLayout tags = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    public limelight (String limelight_name) {
        this.limelight_name = limelight_name;
        tags = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    }
    Angle get_angle_from_target (RawFiducial fid) {
        return Degrees.of(LimelightHelpers.getIMUData(limelight_name).robotYaw - fid.txnc );
    }
    
    
    Optional<RawFiducial[]> pick_three_targets(RawFiducial[] m_fiducials) {
        if (m_fiducials.length < 3) {
            return Optional.empty();
        }
        RawFiducial smallest = pick_furthest_targets(m_fiducials).get()[0];
        RawFiducial biggest = pick_furthest_targets(m_fiducials).get()[1];
        RawFiducial middle = m_fiducials[0]; 
        for (RawFiducial fid : m_fiducials) {
            if (fid.txnc < smallest.txnc &&
                fid.txnc > biggest.txnc &&
                fid != smallest &&
                fid != biggest) {
                middle = fid;
            }else if (fid.txnc < smallest.txnc) {
                smallest = fid;
            }else if (fid.txnc > biggest.txnc){
                biggest = fid;
            }
        }
        return Optional.of(new RawFiducial[] {smallest, middle, biggest});
    }

    
    Optional<RawFiducial[]> pick_furthest_targets(RawFiducial[] m_fiducials) {
        RawFiducial smallest = fiducials[0];
        RawFiducial biggest  = fiducials[0];
        for (RawFiducial fiducial : m_fiducials) {
            if (get_angle_from_target(fiducial).in(Degrees) < get_angle_from_target(smallest).in(Degrees)) {
                smallest = fiducial;
            }else if (get_angle_from_target(fiducial).in(Degrees) > get_angle_from_target(biggest).in(Degrees)){
                biggest = fiducial;

            }
        }
        return Optional.of(new RawFiducial[]{smallest, biggest});
    }

    
    Angle angle_between_two_targets(RawFiducial[] input) {

        return Angle.ofBaseUnits(Math.abs(get_angle_from_target(input[0]).minus(get_angle_from_target(input[1])).in(Degrees)), Degrees);
    }

    Double get_rad_from_chord (double len_of_chord, Angle inscribed_angle) {
        return len_of_chord/(2*Math.sin(inscribed_angle.in(Radians)));
    }
    
    public Optional<Translation2d> getPosition() {
        SmartDashboard.putString("step", "get Pose running");

        fiducials = LimelightHelpers.getRawFiducials(limelight_name);
        try {
        if (fiducials.length == 1) {
            SmartDashboard.putString("step", "One tags seen, asking to pick tags");

            RawFiducial target = fiducials[0];
            Translation3d vector_to_tag3d = tags.getTagPose(target.id).get().getTranslation();
            
            Translation2d vector_to_tag2d = new Translation2d(vector_to_tag3d.getX(), vector_to_tag3d.getY());
            double Limelight_mounting_height = 1;//constants.LimelightMountingHeight;
            double tag_height = tags.getTagPose(target.id).get().getTranslation().getZ();
            double dist_to_tag2d = ( tag_height - Limelight_mounting_height ) / Math.tan(Degrees.of(target.tync /*+ constants.LimelightMountingAngleDeg*/).in(Radians));
            Translation2d vector_from_tag_to_robot = new Translation2d(dist_to_tag2d, new Rotation2d(Degrees.of(-target.txnc+180)));
            return Optional.of(vector_from_tag_to_robot.plus(vector_to_tag2d));
        }
        else if (fiducials.length == 2) {
            SmartDashboard.putString("step", "Two tags seen, asking to pick tags");

                Optional<RawFiducial[]> targets = pick_furthest_targets(fiducials);
                
                Translation2d first_line_first_point = new Translation2d(tags.getTagPose(targets.get()[0].id).get().getTranslation().getX(), tags.getTagPose(targets.get()[0].id).get().getTranslation().getY());
                Translation2d second_line_first_point = new Translation2d(tags.getTagPose(targets.get()[1].id).get().getTranslation().getX(), tags.getTagPose(targets.get()[1].id).get().getTranslation().getY());
                
                Translation2d POI = LineIntersection.calculateIntersectionPoint(
                    first_line_first_point,
                    new Rotation2d(get_angle_from_target(targets.get()[0])),
                    second_line_first_point,
                    new Rotation2d(get_angle_from_target(targets.get()[1])));
                SmartDashboard.putNumber("p1x", LimelightHelpers.getIMUData(limelight_name).robotYaw);
                SmartDashboard.putNumber("p1y", pick_furthest_targets(fiducials).get()[0].txnc);
        
                return Optional.of(new Translation2d(POI.getX(),POI.getY())
            );
            
        

        }
        else if (fiducials.length >= 3){
            SmartDashboard.putString("step", "Three tags seen");

            RawFiducial[] targets = pick_three_targets(fiducials).get();

            Translation2d tag1 = new Translation2d(tags.getTagPose(targets[0].id).get().getTranslation().getX(), tags.getTagPose(targets[0].id).get().getTranslation().getY());
            Translation2d tag2 = new Translation2d(tags.getTagPose(targets[1].id).get().getTranslation().getX(), tags.getTagPose(targets[1].id).get().getTranslation().getY());
            Translation2d tag3 = new Translation2d(tags.getTagPose(targets[2].id).get().getTranslation().getX(), tags.getTagPose(targets[2].id).get().getTranslation().getY());

            robot.tag1display.setLength(tag1.getNorm());
            robot.tag1display.setAngle(tag1.getAngle());


            robot.tag2display.setLength(tag2.getNorm());
            robot.tag2display.setAngle(tag2.getAngle());

            
            robot.tag3display.setLength(tag3.getNorm());
            robot.tag3display.setAngle(tag3.getAngle());

            SmartDashboard.putNumber("tag 1 length", tag1.getNorm());
            SmartDashboard.putNumber("tag 2 length", tag1.getNorm());
            SmartDashboard.putNumber("tag 3 length", tag1.getNorm());

            Translation2d chord1 = new Translation2d (
                tags.getTagPose(targets[0].id).get().getTranslation().minus(tags.getTagPose(targets[1].id).get().getTranslation()).getX(),
                tags.getTagPose(targets[0].id).get().getTranslation().minus(tags.getTagPose(targets[1].id).get().getTranslation()).getY()
            );
            Translation2d chord2 = new Translation2d (
                tags.getTagPose(targets[1].id).get().getTranslation().minus(tags.getTagPose(targets[2].id).get().getTranslation()).getX(),
                tags.getTagPose(targets[1].id).get().getTranslation().minus(tags.getTagPose(targets[2].id).get().getTranslation()).getY()
            );
            double radius1 = get_rad_from_chord(chord1.getNorm(), Degrees.of(targets[0].txnc - targets[1].txnc));
            double radius2 = get_rad_from_chord(chord1.getNorm(), Degrees.of(targets[1].txnc - targets[2].txnc));

            double dist_to_chord1_from_origin = Math.sqrt((radius1*radius1)+((chord2.getNorm()/2)*(chord2.getNorm()/2)));
            double dist_to_chord2_from_origin = Math.sqrt((radius2*radius2)+((chord2.getNorm()/2)*(chord2.getNorm()/2)));
            
            Translation2d midpoint1 = new Translation2d(tag1.getX() + (tag2.getX())/2, tag2.getX() + (tag3.getX())/2);
            Translation2d midpoint2 = new Translation2d(tag2.getX() + (tag3.getX())/2, tag2.getX() + (tag3.getX())/2);

            Translation2d origin1 = midpoint1.minus(new Translation2d(dist_to_chord1_from_origin, 
                new Translation2d(tag1.minus(tag2).getY(), -tag1.minus(tag2).getX()).getAngle()
            ));
            
            Translation2d origin2 = midpoint2.minus(new Translation2d(dist_to_chord2_from_origin, 
                new Translation2d(tag2.minus(tag3).getY(), -tag2.minus(tag3).getX()).getAngle()
            ));
            List<Translation2d> POIs = new ArrayList<>();
            POIs = LineIntersection.getCircleIntersectionPoints(origin1.getX(), origin1.getY(), origin2.getX(), radius1, origin2.getY(), radius2);
            
            Optional<Translation2d> robotPose = Optional.empty();
            for (Translation2d POI : POIs) {
                if(POI.getX()-tag1.getX() < 0.01 && POI.getY()-tag1.getY() < 0.01) {
                    robotPose = Optional.of(POI);
                }
            }
            return robotPose;

        }
        else {
            SmartDashboard.putString("step", "No tags seen");

            return Optional.empty();
        }
        }
            catch (Exception e){
                return Optional.empty();
            }
    }
}

