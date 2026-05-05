package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class commandable_flag {
    private int value = 0;

    public void increment() {
        ++value;
    }

    public void decrement() {
        --value;
    }

    public boolean get() {
        return value != 0;
    }

    public void reset() {
        value = 0;
    }

    public Command inc() {
        return Commands.runOnce( this::increment );
    }

    public Command dec() {
        return Commands.runOnce( this::decrement );
    }

    public Command run(boolean runs_when_disabled) {
        return Commands.startEnd(() -> {
            increment();
        }, () -> {
            decrement();
        }).ignoringDisable(runs_when_disabled);
    }

    public Command run() {
        return run(false);
    }
}
