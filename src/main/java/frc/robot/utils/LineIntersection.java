package frc.robot.utils;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class LineIntersection {
    
    /**
     * Calculates the intersection point of two lines defined by their endpoints.
     * 
     * @param p1 Start point of the first line.
     * @param p2 End point of the first line.
     * @param p3 Start point of the second line.
     * @param p4 End point of the second line.
     * @return The intersection Point2D.Double, or null if lines are parallel or coincident.
     */
    public static Translation2d calculateIntersectionPoint(Translation2d p1, Translation2d p2, Translation2d p3, Translation2d p4) {
        // Line 1 coefficients (a1, b1, c1) from a1*x + b1*y = c1
        double a1 = p2.getY() - p1.getY();
        double b1 = p1.getX() - p2.getX();
        double c1 = a1 * p1.getX() + b1 * p1.getY();

        // Line 2 coefficients (a2, b2, c2)
        double a2 = p4.getY() - p3.getY();
        double b2 = p3.getX() - p4.getX();
        double c2 = a2 * p3.getX() + b2 * p3.getY();

        // Determinant (delta) of the system of equations
        double delta = a1 * b2 - a2 * b1;

        // If delta is 0, the lines are parallel or coincident
        if (delta == 0) {
            return null;
        }

        // Calculate the intersection point coordinates
        double x = (b2 * c1 - b1 * c2) / delta;
        double y = (a1 * c2 - a2 * c1) / delta;


        return new Translation2d(x, y);
    }

    public static Translation2d calculateIntersectionPoint(Translation2d p1, Rotation2d theta1, Translation2d p2, Rotation2d theta2) {
        // creating offsetted points
        Translation2d offseted_point_l1 = new Translation2d(1, theta1).plus(p1);
        Translation2d offseted_point_l2 = new Translation2d(1, theta2).plus(p2);

        return calculateIntersectionPoint( p1, offseted_point_l1, p2, offseted_point_l2 );
    }

    /**
     * Calculates the intersection points of two circles.
     * @param x1 The x coordinate of the first circle's center.
     * @param y1 The y coordinate of the first circle's center.
     * @param r1 The radius of the first circle.
     * @param x2 The x coordinate of the second circle's center.
     * @param y2 The y coordinate of the second circle's center.
     * @param r2 The radius of the second circle.
     * @return A list of intersection points (0, 1, or 2 points).
     */
    
    public static List<Translation2d> getCircleIntersectionPoints(double x1, double y1, double r1, double x2, double y2, double r2) {
        List<Translation2d> intersectionPoints = new ArrayList<>();

        // Calculate the distance between the centers
        double dx = x1 - x2;
        double dy = y1 - y2;
        double d = Math.sqrt(dx * dx + dy * dy);

        // Check for no solutions
        if (d > r1 + r2 || d < Math.abs(r1 - r2) || d == 0) {
            return intersectionPoints; // Return empty list
        }

        // Calculate a and h
        double a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
        double h = Math.sqrt(r1 * r1 - a * a);

        // Calculate the coordinates of the intersection points
        double x0 = x1 + a * (x2 - x1) / d;
        double y0 = y1 + a * (y2 - y1) / d;
        double x3_i = x0 + h * (y2 - y1) / d;
        double y3_i = y0 - h * (x2 - x1) / d;
        double x3_ii = x0 - h * (y2 - y1) / d;
        double y3_ii = y0 + h * (x2 - x1) / d;

        // Add points to the list
        intersectionPoints.add(new Translation2d(x3_i, y3_i));
        if (Math.abs(h) > 1e-9) { // Avoid adding the same point twice if tangent
            intersectionPoints.add(new Translation2d(x3_ii, y3_ii));
        }

        return intersectionPoints;
    }
}