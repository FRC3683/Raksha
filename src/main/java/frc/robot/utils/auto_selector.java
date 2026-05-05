package frc.robot.utils;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.robot;

public class auto_selector {

    private final auto[] autos_bank1;
    private final auto[] autos_bank2;

    private auto selected_auto = new auto("do nothing auto", null, Commands.none());
    private Command auto_command = Commands.none();

    private Optional<Alliance> alliance = Optional.empty();
    private final Consumer<Translation2d> reset_pose;
    
    private final int[] select_buttons = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    private final int bank_size = select_buttons.length;
    private final int shift_button = 10;
    private final boolean default_shift = false;
    private final int default_auto = 0;

    public auto_selector(auto[] bank1, auto[] bank2, Consumer<Translation2d> reset_pose) {
        this.reset_pose = (pose) -> {
            if(pose != null) {
                reset_pose.accept(pose);
            }
        };
        if(bank1.length > bank_size) {
            System.err.println("too many autos in bank1! only the first " + bank_size + " will be available!");
        }
        if(bank2.length > bank_size) {
            System.err.println("too many autos in bank2! only the first " + bank_size + " will be available!");
        }

        autos_bank1 = Arrays.copyOf(bank1, bank_size);
        autos_bank2 = Arrays.copyOf(bank2, bank_size);
        for(int i = bank1.length; i < bank_size; ++i) {
            autos_bank1[i] = new auto();
        }
        for(int i = bank2.length; i < bank_size; ++i) {
            autos_bank2[i] = new auto();
        }
    }

    public void init() {
        select(default_shift, default_auto);

        for(int i = 0; i < bank_size; ++i) {
            final int index = i;
            oi.cmd_xkeys.button(select_buttons[i]).and(DriverStation::isDisabled).onTrue(Commands.runOnce(() -> {
                select(oi.cmd_xkeys.button(shift_button).getAsBoolean(), index);
            }).ignoringDisable(true));
        }

        new Trigger(
            () -> ( DriverStation.getAlliance().isPresent() && alliance.isEmpty() )
                || ( DriverStation.getAlliance().isPresent() && alliance.isPresent() && DriverStation.getAlliance().get() != alliance.get() )
        ).onTrue(Commands.runOnce(() -> {
            reselect();
        }).ignoringDisable(true));
    }

    private void select(boolean shift, int index) {
        var bank = shift ? autos_bank2 : autos_bank1;
        selected_auto = bank[index];
        reselect();
    }

    public Translation2d get_preferred_start_pose() {
        boolean red = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;
        return red ? selected_auto.red_start_pose : field_util.flip( selected_auto.red_start_pose );
    }

    private void reselect() {
        // alliance = DriverStation.getAlliance().orElse(null);
        // if(selected_auto.alliance_dont_care) {
        reset_pose.accept( robot.is_red() ? selected_auto.red_start_pose : field_util.flip( selected_auto.red_start_pose ));
        auto_command = selected_auto.command;
        SmartDashboard.putString("auto", selected_auto.shuffleboard_string);
        // } else if(alliance.isPresent()) {
        //     boolean red = alliance.get() == Alliance.Red;
        //     reset_pose.accept(red ? selected_auto.red_start_pose : selected_auto.blue_start_pose);
        //     auto_command = red ? selected_auto.red_command : selected_auto.blue_command;
        //     SmartDashboard.putString("auto", alliance.get().name() + " " + selected_auto.shuffleboard_string);
        // } else {
        //     // selected auto has different red/blue auto but no driverstation alliance info is available
        //     auto_command = Commands.none();
        //     SmartDashboard.putString("auto", "no alliance do nothing auto");
        // }
    }

    public Command get_auto_command() {
        return auto_command;
    }

    // public Pair<Color8Bit, Color8Bit> get_auto_color() {
    //     return Pair.of(selected_auto.led_color1, selected_auto.led_color2);
    // }



    // private static final Color[] colors = {
    //     Color.kPink, Color.kRed, Color.kYellow, Color.kGreen, Color.kBlue, Color.kPurple, Color.kWhite
    // };

    private static int c1=0, c2=1;

    public static class auto {
        private final String shuffleboard_string;
        // private final boolean alliance_dont_care;
        private final Command command;
        private final Translation2d red_start_pose; // null for don't reset pose on select
        // private Color8Bit led_color1, led_color2;

        public auto( String shuffleboard_string, Translation2d red_start_pose, Command command ) {
            this.shuffleboard_string = shuffleboard_string;
            this.red_start_pose = red_start_pose;
            this.command = command;
            // this.blue_start_pose = blue_start_pose;
            // this.blue_command = blue_command;
            // alliance_dont_care = false;

            // led_color1 = new Color8Bit(colors[c1]);
            // led_color2 = new Color8Bit(colors[c2]);
            // c2++;
            // if(c2 >= colors.length) {
            //     c1++;
            //     c2 = c1+1;
            // }
        }

        // public auto(String shuffleboard_string, Translation2d start_pose, Command command) {
        //     this.shuffleboard_string = shuffleboard_string;
        //     this.red_start_pose = start_pose;
        //     this.red_command = command;
        //     this.blue_start_pose = start_pose;
        //     this.blue_command = command;
        //     alliance_dont_care = true;
        // }

        public auto() {
            this("do nothing auto", new Translation2d(), Commands.none());
        }
    }
}
