package frc.robot;

import frc.robot.utils.field;

public class game_data {
    public static field my_field() {
        if( robot.is_red() ) {
            return constants.red_field;
        }
        return constants.blue_field;
    }
}
