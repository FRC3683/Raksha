package frc.robot.utils;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class fsm_command extends Command {

    public static void example() {
        fsm_command.build_fsm((fsm) -> {
            var hello = fsm.add_entry_state(Commands.print("hello"));
            var wait = fsm.add_state(Commands.print("waiting").andThen(Commands.waitSeconds(1)));
            var idle = fsm.add_state(Commands.print("idle").andThen(Commands.idle()));
            var goodbye = fsm.add_state(Commands.print("goodbye"));

            fsm.add_natural_transition(hello, wait);
            fsm.add_natural_transition(wait, idle);

            fsm.add_transition(() -> Math.random() < 0.05, hello, idle);
            fsm.add_transition(() -> Math.random() > 0.95, goodbye, idle);
            fsm.add_natural_exit(goodbye);
        }).schedule();
    }

    public class transition {
        private BooleanSupplier transition_condition;
        private state to_state;
    }

    public class state {
        private final Command cmd;
        private final ArrayList<transition> transitions = new ArrayList<>(2);
        private BooleanSupplier exit_condition = () -> false;
        private state(Command cmd) { this.cmd = cmd; }
    }

    private ArrayList<state> states = new ArrayList<>(8);
    private ArrayList<transition> global_transitions = new ArrayList<>(8);
    private ArrayList<BooleanSupplier> global_exit_conditions = new ArrayList<>(2);
    private state entry_state = null;
    private state active_state = null;
    private state transition_to = null;

    public fsm_command() {}

    // optional call after adding all states and transitions
    public void optimize() {
        states.trimToSize();
        for(var state : states) {
            state.transitions.trimToSize();
        }
        global_transitions.trimToSize();
        global_exit_conditions.trimToSize();
    }

    public state add_entry_state(Command cmd) {
        entry_state = add_state(cmd);
        return entry_state;
    }

    public static Command build_fsm(Consumer<fsm_command> builder) {
        fsm_command cmd = new fsm_command();
        builder.accept(cmd);
        return cmd;
    }

    public state add_state(Command cmd) {
        CommandScheduler.getInstance().registerComposedCommands(cmd);
        var state = new state(cmd);
        states.add(state);
        addRequirements(cmd.getRequirements());
        return state;
    }

    public void add_exit_condition(state from, BooleanSupplier exit_condition) {
        from.exit_condition = exit_condition;
    }

    public void add_global_exit(BooleanSupplier exit_condition) {
        global_exit_conditions.add(exit_condition);
    }

    public void add_transition(BooleanSupplier condition, state to, state... from) {
        var transition = new transition();
        transition.transition_condition = condition;
        transition.to_state = to;
        for(var from_state : from) {
            from_state.transitions.add(transition);
        }
    }

    public void add_natural_transition(state from, state to) {
        add_transition(from.cmd::isFinished, to, from);
    }

    public void add_natural_exit(state from) {
        add_exit_condition(from, from.cmd::isFinished);
    }

    public void add_transition_from_any(BooleanSupplier condition, state to) {
        var transition = new transition();
        transition.transition_condition = condition;
        transition.to_state = to;
        global_transitions.add(transition);
    }

    @Override
    public void initialize() {
        transition_to = entry_state;
    }

    private void transition_to(state to_state) {
        active_state.cmd.end(true);
        transition_to = to_state;
        // if we immediately set the active state then it will check it's exit conditions before it get's initialized in the next execution cycle
    }

    @Override
    public void execute() {
        if(transition_to != null) {
            active_state = transition_to;
            active_state.cmd.initialize();
            transition_to = null;
        }
        active_state.cmd.execute();
        for(var transition : active_state.transitions) {
            if(transition.transition_condition.getAsBoolean()) {
                transition_to(transition.to_state);
                return;
            }
        }
        for(var transition : global_transitions) {
            if(transition.transition_condition.getAsBoolean()) {
                transition_to(transition.to_state);
                return;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        active_state.cmd.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        for(var condition : global_exit_conditions) {
            if(condition.getAsBoolean()) {
                return true;
            }
        }
        return active_state.exit_condition.getAsBoolean();
    }

}


// from Chief delphi: https://www.chiefdelphi.com/t/2025-wpilib-feedback/500529/111?u=avocadopeel

// // Declare the state machine
// StateMachine auto = new StateMachine();

// // Declare states
// State pathing = auto.addInitialState(drive.followPath());
// State scoring = auto.addState(scorer.score());
// State celebrating = auto.addState(leds.celebrate());

// // Declare transitions between states
// pathing.transitionsTo(scoring).when(atScoringLocation);
// scoring.transitionsTo(celebrating).whenComplete();

// // StateMachine implements Command for seamless
// // integration with the commands API
// RobotModeTriggers.autonomous().onTrue(auto);