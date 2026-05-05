package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config;
import frc.robot.constants;
import frc.robot.utils.configurable;
import frc.robot.utils.hysteresis;
import frc.robot.utils.shotmap;

public class outgive extends SubsystemBase implements configurable {
    public static final outgive instance = new outgive();

    private TalonFX flywheel_leader, flywheel_follower, hood;

    private PositionVoltage hood_position_request = new PositionVoltage(0);
    private VelocityTorqueCurrentFOC flywheel_request = new VelocityTorqueCurrentFOC(0);
    private final Follower flywheel_Follower_req;

    private TorqueCurrentFOC idle_request = new TorqueCurrentFOC( Amps.of( 6 ) ).withMaxAbsDutyCycle( 0.35 );

    private final StatusSignal<AngularVelocity> velocity_signal;
    private final StatusSignal<Angle> hood_deg_signal;

    private boolean extra_juice = false;

    private double target_rpm = 0, target_hood = 0;

    private hysteresis rpm_hysteresis = new hysteresis( 100.0067, 500.0067, false );
    private hysteresis hood_hysteresis = new hysteresis( 2.0067, 5.50067, false );

    private final double min_degrees = constants.turret.flattest_hood_angle.in( Degrees );
    private final double max_degrees = constants.turret.hardstop_hood_angle.in( Degrees );

    private static boolean zeroed = false;

    private outgive() {
        hood = new TalonFX( config.turret_hood_id, config.main_canbus );
        flywheel_leader = new TalonFX( config.turret_flywheel_left_id, config.main_canbus ); 
        flywheel_follower = new TalonFX( config.turret_flywheel_right_id, config.main_canbus );

        frc.robot.robot.m_orchestra.addInstrument(flywheel_leader);
        frc.robot.robot.m_orchestra.addInstrument(flywheel_follower);
        frc.robot.robot.m_orchestra.addInstrument(hood);

        flywheel_Follower_req = new Follower( flywheel_leader.getDeviceID(), MotorAlignmentValue.Opposed );

        velocity_signal = flywheel_leader.getVelocity();
        hood_deg_signal = hood.getPosition();
        hood_deg_signal.setUpdateFrequency( 100 );
        hood.optimizeBusUtilization();
        flywheel_leader.getTorqueCurrent().setUpdateFrequency(100);
        velocity_signal.setUpdateFrequency(100);
        flywheel_leader.optimizeBusUtilization();
        flywheel_follower.optimizeBusUtilization();

        if( Math.abs( hood.getRotorPosition().getValue().in(Rotations) ) < 1 ) {
            hardstop_hood();
        }

        SmartDashboard.putNumber("test_shot", 1650);
        SmartDashboard.putNumber("hood angle", 59 );

        setDefaultCommand( idle() );
    }

    private double get_rpm() {
        return velocity_signal.getValue().in( RPM );
    }

    private double get_hood_deg() {
        return hood_deg_signal.getValue().in( Degrees );
    }

    public boolean isZerod() {
        return zeroed;
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(
            velocity_signal, hood_deg_signal
        );
        rpm_hysteresis.bangbang_refresh( get_rpm(), target_rpm );
        hood_hysteresis.refresh( get_hood_deg(), target_hood );

        SmartDashboard.putBoolean( "outgive_hood_ready", hood_hysteresis.get_value() );
        SmartDashboard.putBoolean( "outgive_rpm_ready", rpm_hysteresis.get_value() );
    }

    public boolean am_i_dead() {
        return Math.abs( get_hood_deg() ) < 5;
    }

    public void hardstop_hood() {
        hood.setPosition( constants.turret.hardstop_hood_angle );
        zeroed = true;
    }

    private void set_position_hood( Angle hood_position ) {
        if (!zeroed)
        {
            hood.stopMotor();
            return;
        }
        
        target_hood = hood_position.in( Degrees );
        double hood_position_deg = MathUtil.clamp( hood_position.in( Degrees ), min_degrees, max_degrees );
        hood.setControl( hood_position_request.withPosition( Degrees.of( hood_position_deg ) ) );
    }

    private void follow() {
        flywheel_follower.setControl( flywheel_Follower_req );
    }

    public Command set_extra_juice( boolean value ) {
        return Commands.runOnce(() -> {
            extra_juice = value;
        });
    }

    public boolean hood_within( Angle target, Angle tol ) {
        return MathUtil.isNear( target.in(Degrees), hood_deg_signal.getValue().in(Degrees), tol.in(Degrees) );
    }

    private void spin( double rpm ) {
        if( DriverStation.isAutonomous() && extra_juice ) {
            rpm *= 1.05;
        }
        target_rpm = rpm;
        flywheel_leader.setControl( flywheel_request.withVelocity( RPM.of( rpm ) ) );
        follow();
    }

    public boolean is_ready() {
        return rpm_hysteresis.get_value() && hood_hysteresis.get_value(); 
    }

    public Trigger ready() {
        return new Trigger( this::is_ready );
    }

    // private void track_target(double turret_angle) { //i have genuinely no idea if this will work and there's probably a better way to do it.
    //     double Trueangle = turret_angle; //Trueangle is redundant if turret_angle doesn't go above 360 or below 0; replace all of Trueangle with turret_angle if so
    //     double Targetangle = 20 +1; //TODO get the actual target angle somehow.
    //     double opposite_target_angle = Targetangle + 180; //this allows the turret to turn either clockwise or counter-clockwise depending on whitch is closer

    //     if(Trueangle > 360){Trueangle = Trueangle - 360;}
    //     else if(Trueangle < 0){Trueangle = Trueangle + 360;} //these are redundant if Trueangle is redundant
        
    //     if (Trueangle > Targetangle && Trueangle < opposite_target_angle){ 
    //         Trueangle = Trueangle - (Targetangle - Trueangle ); //this decreases trueangle if both Targetangle is greater and Trueangle is less than opposite_target_angle
    //     }
    //     else if (Targetangle > Trueangle && Trueangle > opposite_target_angle){
    //         Trueangle = Targetangle + (Trueangle - Targetangle); //this does almost the exact opposite of the above if statement
    //     } 
    //     outgive_turret.setControl( turret_position_request.withPosition( Trueangle ) ); //TODO translate Trueangle to turret_position.
    // }

    public Command auto_zero() {
        Debouncer debounce = new Debouncer(0.04);
        
        return run( () -> {
            hood.setControl( new DutyCycleOut(0.2).withIgnoreSoftwareLimits(true));
        } )
        .until( () -> {
            double velocity = hood.getVelocity().getValue().in(DegreesPerSecond);
            double current = hood.getStatorCurrent().getValue().in(Amps);
            return debounce.calculate(velocity < 0.5 && current > 20);
        } )
        .finallyDo( (boolean interrupted) -> {
            hardstop_hood();
        }).withInterruptBehavior( InterruptionBehavior.kCancelIncoming );
    }

    public Command idle() {
        return run(() -> {
            flywheel_leader.setControl( idle_request );
            follow();
            set_position_hood( constants.turret.idle_hood_angle );
            // hood.stopMotor();
        });
    }

    public Command stop_shooting() {
        return run(() -> {
            flywheel_leader.stopMotor();
            flywheel_follower.stopMotor();
            set_position_hood( constants.turret.idle_hood_angle );
            // hood.stopMotor();
        });
    }

    public Command shoot_test() {
        return run(() -> {
            set_position_hood( Degrees.of( SmartDashboard.getNumber("hood angle", 59.0067 ) ) );
            spin( SmartDashboard.getNumber("test_shot", 1650.0067) );
        });
    }

    public Command hardcode_shot( AngularVelocity rpm, Angle hood_angle ) {
        return run(() -> {
            spin( rpm.in(RPM) );
            set_position_hood( hood_angle );
        });
    }

    public Command auto_shoot() {
        return run(() -> {
          
            var shot = Owl_ware.tracking_shot;
            var trans = new Translation2d( shot.vx, shot.vz );
            var hood = trans.getAngle();
            var mps = trans.getNorm();

            double radps = mps / Units.inchesToMeters(2); // flywheel radius
            double rps = Units.radiansToRotations(radps);
            double rpm = rps * 60;

            set_position_hood( Radians.of( hood.getRadians() ) );
            spin( rpm );
        });
    }

    public Command hood( Angle angle ) {
        return run(() -> {
            flywheel_leader.stopMotor();
            flywheel_follower.stopMotor();
            set_position_hood( angle );
        });
    }


    @Override
    public void configure() {
        hood.getConfigurator().apply( config.turret_cfg.hood_cfg );
        flywheel_leader.getConfigurator().apply( config.turret_cfg.left_flywheel_cfg );
        flywheel_follower.getConfigurator().apply( config.turret_cfg.left_flywheel_cfg ); // current limits
        // hood.setPosition( constants.turret.hardstop_hood_angle );
    }


/*
    public Command unjam() {
        return run(() -> {
            set_position( true );
            spin( -19 );
        });
    }

    public Command stow_spin() {
        return run(() -> {
            set_position( false );
            spin( 19 );
        });
    } 
    */
}
