package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Owl_ware;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.outgive;
import frc.robot.utils.field_util;

public final class debug {
    public static void add_dashboard_commands(robot robot) {
        SmartDashboard.putData("reset pose 5,5", Commands.runOnce(() -> {
            robot.swerve.reset_pose(new Pose2d(5, 5, robot.swerve.get_heading()));
        }).ignoringDisable(true));

        SmartDashboard.putData("reset pose lob", Commands.runOnce(() -> {
            robot.swerve.reset_pose(new Pose2d(field_util.field_size_m.getX()/2, field_util.field_size_m.getY()/2, robot.swerve.get_heading()));
        }).ignoringDisable(true));

        SmartDashboard.putData("coast stuff", Commands.runOnce(() -> {
            robot.swerve.coast();
        }).ignoringDisable(true));

        SmartDashboard.putData("zero intake", Commands.runOnce(() -> {
            intake.instance.hardstop();
        }).ignoringDisable(true));

        SmartDashboard.putData("bumper intake", Commands.runOnce(() -> {
            intake.instance.bumper_hardstop();
        }).ignoringDisable(true));

        SmartDashboard.putData("zero hood", Commands.runOnce(() -> {
            outgive.instance.hardstop_hood();
        }).ignoringDisable(true));

        SmartDashboard.putData("zero turret", Commands.runOnce(() -> {
            Owl_ware.instance.zero().schedule();
        }).ignoringDisable(true));

            SmartDashboard.putData("auto zero hood", Commands.runOnce(() -> {
            outgive.instance.auto_zero().schedule();
        }).ignoringDisable(true));

         SmartDashboard.putData("auto zero intake", Commands.runOnce(() -> {
            intake.instance.auto_zero().schedule();
        }).ignoringDisable(true));

    }


}
