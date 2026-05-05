package frc.robot.utils;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;

public final class sotm {
    private sotm() {}

    static double resultantMagnitude(double vx1, double vy1, double vx2, double vy2){

        double resultant_x = vx2 - vx1;
        double resultant_y = vy2 - vy1;

        double resultant = Math.sqrt(Math.pow(resultant_x, 2) + Math.pow(resultant_y, 2));

        return resultant;
    }

    static double resultantDirection(double vx1, double vy1, double vx2, double vy2){

        double resultant_x = vx2 - vx1;
        double resultant_y = vy2 - vy1;

        double thetaRAD = Math.atan2(resultant_y, resultant_x);
        double thetaDeg = thetaRAD*(180.0/Math.PI);

        return thetaDeg;
    }

    public static  Pair<LinearVelocity, Rotation2d> get_shot(ChassisSpeeds robot, Pair<LinearVelocity, Rotation2d> still_shot){


        double Vx = robot.vxMetersPerSecond;
        double Vy = robot.vyMetersPerSecond;

        double shot_velocity = still_shot.getFirst().in(MetersPerSecond);
        double shot_angle = still_shot.getSecond().getRadians();

        double shot_velocity_x = shot_velocity*Math.cos(shot_angle);
        double shot_velocity_y = shot_velocity*Math.sin(shot_angle);

        double magnitude_mps = resultantMagnitude(Vx, Vy, shot_velocity_x, shot_velocity_y);
        double angle_degrees = resultantDirection(Vx, Vy, shot_velocity_x, shot_velocity_y);
        return Pair.of( MetersPerSecond.of(magnitude_mps), Rotation2d.fromDegrees(angle_degrees) );
    }
}