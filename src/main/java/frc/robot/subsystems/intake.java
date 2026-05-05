package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config;
import frc.robot.constants;
import frc.robot.leds;
import frc.robot.utils.configurable;
import frc.robot.utils.math_utils;

public class intake extends SubsystemBase implements configurable {
    // enum state {
    //     STOWED, DEPLOYED, JOSTLE
    // }

    public static final intake instance = new intake();

    private TalonFX intake_arm;
    private TalonFX roller, roller_slave;

    Timer vomit_timer = new Timer();

    private DynamicMotionMagicVoltage arm_position_request = new DynamicMotionMagicVoltage(0, 0, 8);
    private MotionMagicVoltage lame_arm_position_request = new MotionMagicVoltage(0);
    private PositionVoltage raw_arm_position_request = new PositionVoltage(0);
    private VelocityTorqueCurrentFOC roller_request = new VelocityTorqueCurrentFOC(0);
    private VoltageOut roller_openloop = new VoltageOut(0).withEnableFOC(true);
    private Follower roller_follow;

    private TorqueCurrentFOC jostle_req = new TorqueCurrentFOC( Amps.of( 85 ) ).withMaxAbsDutyCycle(0.3);

    private StatusSignal<AngularVelocity> roller_vel;
    private StatusSignal<Angle> arm_pos;
    private StatusSignal<Current> roller_current;

    private Debouncer stall_debouncer = new Debouncer( 0.15 );
    private boolean is_stalling = false;

    private Timer jostle_timer = new Timer();

    private boolean is_auto_limits = true;

    public static boolean zeroed = false;
    public static boolean hopper_deployed = false;

    // state my_state = state.DEPLOYED;

    private intake() {
        intake_arm = new TalonFX( config.intake_arm_id, config.main_canbus );
        roller = new TalonFX( config.intake_roller_id, config.main_canbus );
        roller_slave = new TalonFX( config.intake_roller_slave_id, config.main_canbus );

        roller_follow = new Follower( roller.getDeviceID(), MotorAlignmentValue.Aligned );


        roller_vel = roller.getVelocity();
        roller_current = roller.getStatorCurrent();

        roller_vel.setUpdateFrequency(100);
        roller_current.setUpdateFrequency(100);
        roller.getMotorVoltage().setUpdateFrequency(100);
        arm_pos = intake_arm.getPosition();
        arm_pos.setUpdateFrequency(50);

        roller.optimizeBusUtilization();
        roller_slave.optimizeBusUtilization();
        intake_arm.optimizeBusUtilization();

        // set_position( state.DEPLOYED );
        setDefaultCommand( idle() );
    }

    public Command auto_zero() {
        Debouncer debounce = new Debouncer(0.08);
        
        return run( () -> {
            intake_arm.setControl( new DutyCycleOut(-0.3).withIgnoreSoftwareLimits(true));
            roller.set(-0.1);
        } )
        .until( () -> {
            double velocity = intake_arm.getVelocity().getValue().in(DegreesPerSecond);
            double current = intake_arm.getStatorCurrent().getValue().in(Amps);
            return debounce.calculate(velocity < 0.5 && current > 40);
        } )
        .finallyDo( (boolean interrupted) -> {
            intake_arm.stopMotor();
            intake_arm.setPosition( constants.intake.deployed.minus(Degrees.of(4)) );
            zeroed = true;
            hopper_deployed = true;
        }).withInterruptBehavior( InterruptionBehavior.kCancelIncoming );
    }

    public void apply_auto_limits() {
        if( is_auto_limits ) {
            return;
        }
        is_auto_limits = true;
        roller.getConfigurator().apply( config.intake_cfg.roller_auto_limits );
        roller_slave.getConfigurator().apply( config.intake_cfg.roller_auto_limits );
    }

    public void apply_tele_limits() {
        if( ! is_auto_limits ) {
            return;
        }
        is_auto_limits = false;
        roller.getConfigurator().apply( config.intake_cfg.roller_tele_limits );
        roller_slave.getConfigurator().apply( config.intake_cfg.roller_tele_limits );
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
            roller_vel, roller_current, arm_pos
        );
        is_stalling = stall_debouncer.calculate( Math.abs( roller_vel.getValue().in(RPM) ) < 50 && roller_current.getValue().in(Amps) > 60 );

        if( arm_pos.getValue().gt(Degrees.of(80)) ) {
            hopper_deployed = false;
        }
        // set_position( my_state );
    }

    public void hardstop() {
        intake_arm.setPosition( constants.intake.hardstop );
        zeroed = true;
        hopper_deployed = false;
    }

    public void bumper_hardstop() {
        intake_arm.setPosition( constants.intake.deployed );
        zeroed = true;
        hopper_deployed = true;
    }

    private void set_angle( Angle angle, boolean slow ) {
        if (!zeroed)
        {
            intake_arm.stopMotor();
        }
        else
        {
        // CANIVORE BREAK DO THIS:
        // if( slow ) {
        //     intake_arm.setControl( lame_arm_position_request.withPosition( angle ) );
        // } else {
        //     intake_arm.setControl( raw_arm_position_request.withPosition( angle ) );
        // }
            intake_arm.setControl( arm_position_request.withPosition( angle ).withVelocity( slow ? 1.0 : 1.5 ) );
        }
    }

    public Command auton_depot() {
        return run(() -> {
            spin( constants.intake.intake_speed );
            set_angle( constants.intake.depot, false );
        });
    }

    public Command stop_em_from_floppen() {
        return run(() -> {
            roller.stopMotor();
            roller_slave.stopMotor();
            set_angle( constants.intake.up_a_bit, false );
        });
    }

    // private void set_position( state newstate ) {
    //     Angle position = constants.intake.stowed;
    //     my_state = newstate;
    //     if( my_state == state.DEPLOYED ) {
    //         position = constants.intake.deployed;
    //     }
    //     if( my_state == state.STOWED ) {
    //         position = constants.intake.stowed;
    //     }
    //     set_angle( position, false );
    // }

    private void spin( AngularVelocity intake_speed ) {
        if( !neck.instance.jammed() ) {
            if( leds.flag_unjamming.get() ) {
                leds.flag_unjamming.decrement();
            }
            spin_raw( intake_speed );
            return;
        }
        if( !leds.flag_unjamming.get() ) {
            leds.flag_unjamming.increment();
        }
        spin_raw( intake_speed );
    }

    private void spin_raw( AngularVelocity speed ) {
        roller.setControl( roller_openloop.withOutput( 12 * speed.in( RPM ) / 5800 ) );
        roller_slave.setControl( roller_follow );
    }

    public Command jostle() {
        return jostle(false);
    }

    public Command jostle(boolean auto_vomit) {
        TorqueCurrentFOC m_torqueRequest = new TorqueCurrentFOC(0);
        return 
        Commands.runOnce(() -> {
            jostle_timer.restart();
        }).alongWith(
            run(() -> {
                
                double target_pos = 65;
                // if( Math.floor( jostle_timer.get() * 4 ) % 2 == 0 ) {
                //     target_pos = 0;
                // }
                double pos = arm_pos.getValue().in(Degrees);
                double err = target_pos - pos;
                // pos = math_utils.clamp( -15, 65, pos );
                // double pos_in_standard = pos + 15;
                double kP = 0.50067;

            
                intake_arm.setControl(m_torqueRequest.withOutput(err * kP));

                if (auto_vomit && neck.instance.jammed()) {
                    spin(constants.intake.intake_speed.unaryMinus());
                } else {
                    roller.setControl( jostle_req );
                }
                roller_slave.setControl( roller_follow );
            })
        );
    }

    public Command auton_jostle() {
        return Commands.repeatingSequence(
            jostle(true).withTimeout( 1.50067 ),
            succ().withTimeout( 0.20067 )
        );
    }

    public Command pre_depot_jostle() {
        return jostle();
        // return Commands.runOnce(() -> {
        //     jostle_timer.restart();
        // }).alongWith(
        //     run(() -> {
        //         spin( constants.intake.intake_speed );
        //         if( Math.round( jostle_timer.get() * 4 ) % 2 == 0 ) {
        //             set_angle( constants.intake.deployed, false );
        //         } else {
        //             set_angle( constants.intake.jostle2, false );
        //         }
        //     })
        // );
    }

    // public Command toggle_deploy() {
    //     return Commands.runOnce(() -> {
    //         deployed = !deployed;
    //         set_position( deployed );
    //     });
    // }

    // public Command set_deploy_state( boolean deployed ) {
    //     return Commands.runOnce(() -> {
    //         this.deployed = deployed;
    //         set_position( deployed );
    //     });
    // }

    public Command idle() {
        return Commands.either(
            run(() -> {
                if( arm_pos.getValue().in( Degrees ) > 0 ) {
                    spin( constants.intake.intake_speed.times( -0.60067 ) );
                } else {
                    spin( DegreesPerSecond.of( 0 ) );
                }
                set_angle( constants.intake.deployed, false );
                if( arm_pos.getValue().in(Degrees) < 10.0067 ) {
                    hopper_deployed = true;
                }
            }),
            run(() -> {
                roller.stopMotor();
                roller_slave.stopMotor();
                set_angle( constants.intake.deployed, false );
            }),
            () -> arm_pos.getValue().in( Degrees ) > 40.0067
        );
    }

    boolean smart_vomit = false;

    public Command succ() {
        return Commands.runOnce(() -> {
            vomit_timer.restart();
        }).alongWith( run(() -> {
            if( is_stalling && !smart_vomit ) {
                smart_vomit = true;
                vomit_timer.restart();
            }
            if( smart_vomit && vomit_timer.hasElapsed( 0.50067 ) ) {
                smart_vomit = false;
            }
            set_angle( constants.intake.deployed, false );
            spin( smart_vomit ? constants.intake.intake_speed.unaryMinus() : constants.intake.intake_speed );
        }) );
    }


    public Command vomit() {
        return run(() -> {
            set_angle( constants.intake.deployed, false );
            spin( constants.intake.intake_speed.unaryMinus() );

        });
    }

    // public Command stow_spin() {
    //     return run(() -> {
    //         spin( 19 );
    //     });
    // }

    @Override
    public void configure() {
        intake_arm.getConfigurator().apply( config.intake_cfg.arm_cfg );
        roller.getConfigurator().apply( config.intake_cfg.roller_cfg );
        roller_slave.getConfigurator().apply( config.intake_cfg.roller_cfg ); // current limits
    }
}
